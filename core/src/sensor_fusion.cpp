#include "sensor_fusion.hpp"
#include "rtklib.h"
#include <cmath>
#include <Eigen/Geometry>
#include <android/log.h>

#define TAG "RTK_Native"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

namespace rtklib_localization {

SensorFusionEKF::SensorFusionEKF() {
    x_.resize(15);
    x_.setZero();
    
    P_.resize(15, 15);
    P_.setIdentity();
    
    consecutive_gnss_rejections_ = 0;

    // Default Process Noise
    Q_diag_.resize(15);
    Q_diag_ << 0.01, 0.01, 0.01,   // Pos
               0.01, 0.01, 0.01,   // Ori
               0.05, 0.05, 0.05,   // Vel
               1e-4, 1e-4, 1e-4,   // Gyro Bias
               1e-3, 1e-3, 1e-3;   // Accel Bias
}

Eigen::Vector3d SensorFusionEKF::wgs84ToEnu(double lat, double lon, double alt) const {
    if (!has_origin_) return Eigen::Vector3d::Zero();

    double pos_origin[3] = {origin_lat_, origin_lon_, origin_alt_};
    double pos_curr[3] = {lat * D2R, lon * D2R, alt};

    double r_origin[3], r_curr[3];
    pos2ecef(pos_origin, r_origin);
    pos2ecef(pos_curr, r_curr);

    double delta_r[3] = {r_curr[0] - r_origin[0], r_curr[1] - r_origin[1], r_curr[2] - r_origin[2]};
    double enu_arr[3];
    ecef2enu(pos_origin, delta_r, enu_arr);

    return Eigen::Vector3d(enu_arr[0], enu_arr[1], enu_arr[2]);
}

Eigen::Vector3d SensorFusionEKF::enuToWgs84(const Eigen::Vector3d& enu) const {
    if (!has_origin_) return Eigen::Vector3d::Zero();

    double pos_origin[3] = {origin_lat_, origin_lon_, origin_alt_};
    double enu_arr[3] = {enu.x(), enu.y(), enu.z()};

    double delta_r[3];
    enu2ecef(pos_origin, enu_arr, delta_r);

    double r_origin[3];
    pos2ecef(pos_origin, r_origin);

    double r_curr[3] = {r_origin[0] + delta_r[0], r_origin[1] + delta_r[1], r_origin[2] + delta_r[2]};
    double pos_curr[3];
    ecef2pos(r_curr, pos_curr);

    return Eigen::Vector3d(pos_curr[0] * R2D, pos_curr[1] * R2D, pos_curr[2]);
}

void SensorFusionEKF::initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    initializeInternal(initial_state, initial_cov);
}

void SensorFusionEKF::initializeInternal(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov) {
    x_ = initial_state;
    P_ = initial_cov;
    is_initialized_ = true;
    consecutive_gnss_rejections_ = 0;
}

void SensorFusionEKF::predict(double current_time, const Eigen::Vector3d& linear_accel, const Eigen::Vector3d& angular_vel) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    predictInternal(current_time, linear_accel, angular_vel);
}

void SensorFusionEKF::predictInternal(double current_time, const Eigen::Vector3d& linear_accel, const Eigen::Vector3d& angular_vel) {
    if (!is_initialized_) return;

    double dt = current_time - last_update_time_;
    if (dt <= 0.0) return;
    if (dt > 1.0) { // Large gap
        last_update_time_ = current_time;
        return;
    }

    // --- 1. State Propagation ---
    integrateStateInternal(dt, linear_accel, angular_vel);

    // --- 2. Covariance Propagation ---
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(15, 15);

    double roll = x_(3), pitch = x_(4), yaw = x_(5);
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yawAngle * pitchAngle * rollAngle).matrix();

    F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(6, 3) = -R * skew(linear_accel - x_.segment<3>(12)) * dt;
    F.block<3, 3>(6, 12) = -R * dt;
    F.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity() * dt;

    Eigen::MatrixXd Q = Q_diag_.asDiagonal();
    P_ = F * P_ * F.transpose() + Q * dt;

    last_update_time_ = current_time;
}

void SensorFusionEKF::integrateStateInternal(double dt, const Eigen::Vector3d& linear_accel, const Eigen::Vector3d& angular_vel) {
    Eigen::Vector3d unbias_gyro = angular_vel - x_.segment<3>(9);
    Eigen::Vector3d unbias_accel = linear_accel - x_.segment<3>(12);

    // Orientation Integration
    x_.segment<3>(3) += unbias_gyro * dt;

    double roll = x_(3), pitch = x_(4), yaw = x_(5);
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yawAngle * pitchAngle * rollAngle).matrix();

    Eigen::Vector3d gravity_nav(0, 0, -9.81);
    Eigen::Vector3d acc_nav = R * unbias_accel + gravity_nav;

    x_.segment<3>(0) += x_.segment<3>(6) * dt + 0.5 * acc_nav * dt * dt;
    x_.segment<3>(6) += acc_nav * dt;
}

void SensorFusionEKF::updateGNSS(const MeasurementGNSS& gnss) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!has_origin_) {
        origin_lat_ = gnss.pos_enu.x() * D2R;
        origin_lon_ = gnss.pos_enu.y() * D2R;
        origin_alt_ = gnss.pos_enu.z();
        has_origin_ = true;
    }

    Eigen::Vector3d pos_enu = wgs84ToEnu(gnss.pos_enu.x(), gnss.pos_enu.y(), gnss.pos_enu.z());

    if (!is_initialized_) {
        Eigen::VectorXd init_x = Eigen::VectorXd::Zero(15);
        init_x.segment<3>(0) = pos_enu;
        if (gnss.has_velocity) init_x.segment<3>(6) = gnss.vel_enu;

        // Tightened initial covariance
        Eigen::MatrixXd init_P = Eigen::MatrixXd::Identity(15, 15) * 1.0;
        init_P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1.0;  // Pos
        init_P.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 5.0;  // Ori
        init_P.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1.0;  // Vel
        init_P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.01; // Gyro Bias
        init_P.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.01; // Accel Bias

        initializeInternal(init_x, init_P);
        last_update_time_ = gnss.timestamp;
        return;
    }

    predictInternal(gnss.timestamp, last_accel_, last_gyro_);

    // Observation Logic
    int m = gnss.has_velocity ? 6 : 3;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, 15);
    Eigen::VectorXd z(m);

    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Observe Position
    z.segment<3>(0) = pos_enu;

    if (gnss.has_velocity) {
        H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(); // Observe Velocity
        z.segment<3>(3) = gnss.vel_enu;
    }

    Eigen::VectorXd y = z - H * x_; // Innovation

    // --- PHYSICAL REALITY CHECK ---
    double jump_distance = y.head<3>().norm();

    if (jump_distance > 50.0) {
        // 1. REJECT IMPOSSIBLE JUMPS (Likely Network Location artifacts)
        consecutive_gnss_rejections_++;
        LOGD("GNSS Jump physically impossible (%.2f m). Ignoring point (Count: %d).",
             jump_distance, consecutive_gnss_rejections_);

        if (consecutive_gnss_rejections_ < 5) {
            return;
        }

        LOGD("Too many consecutive GNSS rejections. Forcing recovery snap.");
        // Fall through to hard-snap logic below
    } else {
        // We have a valid point (or a multipath snap), reset counter
        consecutive_gnss_rejections_ = 0;
    }

    if (jump_distance > 10.0) {
        // 2. HARD-SNAP MULTIPATH BOUNCES (10m - 50m) OR RECOVERY SNAP
        LOGD("GNSS Snap Triggered (%.2f m). Hard-snapping state.", jump_distance);

        // Force position directly to the new GNSS coordinate
        x_.segment<3>(0) = pos_enu;

        // Snap velocity directly to GNSS Doppler speed (or zero if vel=NO)
        if (gnss.has_velocity) {
            x_.segment<3>(6) = gnss.vel_enu;
        } else {
            x_.segment<3>(6).setZero();
        }

        // Kill the acceleration bias integration
        x_.segment<3>(12).setZero();

        // Bypass the standard Kalman update!
        return;
    }

    Eigen::MatrixXd R_meas;
    if (gnss.has_velocity) {
        R_meas = Eigen::MatrixXd::Identity(6, 6);
        R_meas.block<3, 3>(0, 0) = gnss.covariance;
        R_meas.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.5; // Trust GNSS velocity
    } else {
        R_meas = gnss.covariance;
    }

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_meas;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // --- THE DECOUPLING FIX ---
    // Force the filter to NEVER update Pitch, Roll, or Yaw based on GPS wobbles.
    // This prevents GPS inaccuracies from causing gravity leaks.
    K.row(3).setZero(); // Protect Roll
    K.row(4).setZero(); // Protect Pitch
    K.row(5).setZero(); // Protect Yaw

    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(15, 15) - K * H) * P_;
}

void SensorFusionEKF::updateIMU(const MeasurementIMU& imu) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!is_initialized_) return;

    if (!is_aligned_) {
        // ... (alignment logic remains same)
        accel_sum_ += imu.lin_accel;
        gyro_sum_ += imu.ang_vel;
        alignment_samples_++;

        if (alignment_samples_ >= ALIGNMENT_THRESHOLD) {
            Eigen::Vector3d avg_accel = accel_sum_ / alignment_samples_;
            x_.segment<3>(9) = gyro_sum_ / alignment_samples_;

            double g = avg_accel.norm();
            if (g > 0.1) {
                double pitch = -asin(avg_accel.x() / g);
                double roll = atan2(avg_accel.y(), avg_accel.z());
                x_(3) = roll;
                x_(4) = pitch;
            }
            x_(5) = 0.0;
            is_aligned_ = true;
            last_update_time_ = imu.timestamp;
        }
        return;
    }

    // --- ZUPT (Zero Velocity Update) ---
    // Detect if device is stationary to prevent drift integration.
    double accel_noise = std::abs(imu.lin_accel.norm() - 9.81);
    double gyro_mag = imu.ang_vel.norm();

    if (accel_noise < 0.3 && gyro_mag < 0.05) {
        x_.segment<3>(6).setZero();  // Clamp Velocity
        x_.segment<3>(12).setZero(); // Clamp Accel Bias integration
    }

    predictInternal(imu.timestamp, imu.lin_accel, imu.ang_vel);
    last_accel_ = imu.lin_accel;
    last_gyro_ = imu.ang_vel;
}

Eigen::VectorXd SensorFusionEKF::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return x_;
}

Eigen::MatrixXd SensorFusionEKF::getCovariance() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return P_;
}

Eigen::Matrix3d SensorFusionEKF::skew(const Eigen::Vector3d& v) const {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}

} // namespace rtklib_localization
