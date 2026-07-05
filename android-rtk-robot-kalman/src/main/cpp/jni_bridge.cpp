#include <jni.h>
#include <memory>
#include <android/log.h>
#include "sensor_fusion.hpp"

#define TAG "RTK_Native"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

using namespace rtklib_localization;

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativeCreateEngine(JNIEnv* env, jobject thiz) {
    auto engine = new SensorFusionEKF();
    return reinterpret_cast<jlong>(engine);
}

JNIEXPORT void JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativeDestroyEngine(JNIEnv* env, jobject thiz, jlong engine_ptr) {
    if (engine_ptr != 0) {
        delete reinterpret_cast<SensorFusionEKF*>(engine_ptr);
    }
}

JNIEXPORT void JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativePushIMU(JNIEnv* env, jobject thiz, jlong engine_ptr,
                                                                       jdouble timestamp, jdouble ax, jdouble ay, jdouble az,
                                                                       jdouble wx, jdouble wy, jdouble wz) {
    auto* engine = reinterpret_cast<SensorFusionEKF*>(engine_ptr);
    if (engine) {
        MeasurementIMU imu{
                timestamp,
                Eigen::Vector3d(ax, ay, az),
                Eigen::Vector3d(wx, wy, wz)
        };
        engine->updateIMU(imu);
    }
}

JNIEXPORT void JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativeGetState(JNIEnv* env, jobject thiz, jlong engine_ptr, jdoubleArray out_array) {
    auto engine = reinterpret_cast<SensorFusionEKF*>(engine_ptr);
    if (!engine || !out_array) return;

    Eigen::VectorXd state = engine->getState();

    // Convert ENU back to WGS84 for the UI
    Eigen::Vector3d pos_enu = state.segment<3>(0);
    Eigen::Vector3d pos_wgs84 = engine->enuToWgs84(pos_enu);

    state(0) = pos_wgs84.x(); // Latitude
    state(1) = pos_wgs84.y(); // Longitude
    state(2) = pos_wgs84.z(); // Altitude

    env->SetDoubleArrayRegion(out_array, 0, 15, state.data());
}


JNIEXPORT void JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativePushGNSS(JNIEnv* env, jobject thiz, jlong engine_ptr,
                                                                        jdouble timestamp, jdouble lat, jdouble lon, jdouble alt,
                                                                        jdouble vx, jdouble vy, jdouble vz, jboolean has_velocity,
                                                                        jdoubleArray jcov) {
    auto* engine = reinterpret_cast<SensorFusionEKF*>(engine_ptr);
    if (engine && jcov) {
        jdouble* cov_ptr = env->GetDoubleArrayElements(jcov, nullptr);
        Eigen::Matrix3d covariance = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(cov_ptr);
        env->ReleaseDoubleArrayElements(jcov, cov_ptr, JNI_ABORT);

        MeasurementGNSS gnss{
                timestamp,
                Eigen::Vector3d(lat, lon, alt),
                Eigen::Vector3d(vx, vy, vz),
                static_cast<bool>(has_velocity),
                covariance
        };
        engine->updateGNSS(gnss);
    }
}

JNIEXPORT void JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativePushOrientation(JNIEnv* env, jobject thiz, jlong engine_ptr,
                                                                               jdouble timestamp, jdouble roll, jdouble pitch, jdouble yaw) {
    auto* engine = reinterpret_cast<SensorFusionEKF*>(engine_ptr);
    if (engine) {
        MeasurementOrientation orient{timestamp, roll, pitch, yaw};
        engine->updateOrientation(orient);
    }
}

JNIEXPORT void JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativePushRawObservations(JNIEnv* env, jobject thiz, jlong engine_ptr,
                                                                                   jdouble timestamp, jobjectArray jobs_array) {
    auto* engine = reinterpret_cast<SensorFusionEKF*>(engine_ptr);
    if (engine && jobs_array) {
        int n = env->GetArrayLength(jobs_array);
        std::vector<std::vector<double>> raw_data;
        for (int i = 0; i < n; ++i) {
            auto jdouble_array = (jdoubleArray)env->GetObjectArrayElement(jobs_array, i);
            int m = env->GetArrayLength(jdouble_array);
            jdouble* elements = env->GetDoubleArrayElements(jdouble_array, nullptr);
            std::vector<double> row(elements, elements + m);
            raw_data.push_back(row);
            env->ReleaseDoubleArrayElements(jdouble_array, elements, JNI_ABORT);
            env->DeleteLocalRef(jdouble_array);
        }
        engine->updateRawObservations(timestamp, raw_data);
    }
}

} // extern "C"
