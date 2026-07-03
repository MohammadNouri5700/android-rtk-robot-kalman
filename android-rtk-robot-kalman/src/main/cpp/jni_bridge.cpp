#include <jni.h>
#include <memory>
#include "sensor_fusion.hpp"

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

JNIEXPORT jdoubleArray JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativeGetState(JNIEnv* env, jobject thiz, jlong engine_ptr) {
    auto engine = reinterpret_cast<SensorFusionEKF*>(engine_ptr);
    if (!engine) return nullptr;

    Eigen::VectorXd state = engine->getState();

    // Convert ENU back to WGS84 for the UI
    Eigen::Vector3d pos_enu = state.segment<3>(0);
    Eigen::Vector3d pos_wgs84 = engine->enuToWgs84(pos_enu);

    state(0) = pos_wgs84.x(); // Latitude
    state(1) = pos_wgs84.y(); // Longitude
    state(2) = pos_wgs84.z(); // Altitude

    jdoubleArray result = env->NewDoubleArray(15);
    if (result == nullptr) return nullptr;

    env->SetDoubleArrayRegion(result, 0, 15, state.data());
    return result;
}


JNIEXPORT void JNICALL
Java_com_mohammadnouri5700_rtkrobotkalman_RtkRobotEngine_nativePushGNSS(JNIEnv* env, jobject thiz, jlong engine_ptr,
                                                                        jdouble timestamp, jdouble lat, jdouble lon, jdouble alt,
                                                                        jdouble cov_x, jdouble cov_y, jdouble cov_z) {
    auto* engine = reinterpret_cast<SensorFusionEKF*>(engine_ptr);
    if (engine) {
        MeasurementGNSS gnss{
                timestamp,
                Eigen::Vector3d(lat, lon, alt),
                Eigen::Matrix3d::Identity()
        };

        gnss.covariance(0, 0) = cov_x;
        gnss.covariance(1, 1) = cov_y;
        gnss.covariance(2, 2) = cov_z;

        engine->updateGNSS(gnss);
    }
}

} // extern "C"
