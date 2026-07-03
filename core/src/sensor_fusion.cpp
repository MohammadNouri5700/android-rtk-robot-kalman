#include "sensor_fusion.hpp"
#include "rtklib.h"
#include <cmath>
#include <Eigen/Geometry>

namespace rtklib_localization {

SensorFusionEKF::SensorFusionEKF() {
    x_.resize(15);
    x_.setZero();
    
    P_.resize(15, 15);
    P_.setIdentity();
    
    Q_.resize(15, 15);
    Q_.setIdentity() * 0.05;
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
}

void SensorFusionEKF::predict(double current_time) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    predictInternal(current_time);
}

void SensorFusionEKF::predictInternal(double current_time) {
    if (!is_initialized_) return;

    double dt = current_time - last_update_time_;
    if (dt <= 0.0) return;

    integrateStateInternal(dt);

    // State Transition Jacobian (F) simplification for Kinematic Process Model
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(15, 15);
    F.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3) * dt;
    F.block<3, 3>(6, 12) = Eigen::MatrixXd::Identity(3, 3) * dt;
    F.block<3, 3>(3, 9) = Eigen::MatrixXd::Identity(3, 3) * dt;

    P_ = F * P_ * F.transpose() + Q_ * dt;
    last_update_time_ = current_time;
}

void SensorFusionEKF::integrateStateInternal(double dt) {
    x_.segment<3>(0) += x_.segment<3>(6) * dt + 0.5 * x_.segment<3>(12) * dt * dt;
    x_.segment<3>(6) += x_.segment<3>(12) * dt;
    x_.segment<3>(3) += x_.segment<3>(9) * dt;
}

void SensorFusionEKF::updateGNSS(const MeasurementGNSS& gnss) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // First valid GNSS point establishes the ENU origin
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
        initializeInternal(init_x, Eigen::MatrixXd::Identity(15, 15) * 1.0);
        last_update_time_ = gnss.timestamp;
        return;
    }

    predictInternal(gnss.timestamp);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Vector3d z = pos_enu;
    Eigen::Vector3d y = z - H * x_; // Innovation
    
    Eigen::MatrixXd S = H * P_ * H.transpose() + gnss.covariance;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
    P_ = (I - K * H) * P_;
}

void SensorFusionEKF::updateIMU(const MeasurementIMU& imu) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!is_initialized_) return;

    predictInternal(imu.timestamp);

    // 1. Gravity Compensation
    // Rotate measured acceleration from body frame to navigation frame (ENU)
    double roll = x_(3), pitch = x_(4), yaw = x_(5);
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yawAngle * pitchAngle * rollAngle).matrix();

    // acc_nav = R * acc_body - gravity_nav
    // Gravity in ENU is [0, 0, -9.81]. So we subtract [0, 0, -9.81] which is adding [0, 0, 9.81]
    // However, IMU measures (a - g). So acc_body = R^T * (a_nav - g_nav)
    // a_nav = R * acc_body + g_nav
    Eigen::Vector3d gravity_nav(0, 0, -9.81);
    Eigen::Vector3d acc_nav = R * imu.lin_accel + gravity_nav;

    // 2. EKF Update
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 15);
    H.block<3, 3>(0, 9) = Eigen::MatrixXd::Identity(3, 3);
    H.block<3, 3>(3, 12) = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Matrix<double, 6, 1> z;
    z << imu.ang_vel, acc_nav;

    Eigen::Matrix<double, 6, 1> y = z - H * x_;
    
    Eigen::MatrixXd R_cov = Eigen::MatrixXd::Identity(6, 6) * 0.02;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_cov;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
    P_ = (I - K * H) * P_;
}

Eigen::VectorXd SensorFusionEKF::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return x_;
}

Eigen::MatrixXd SensorFusionEKF::getCovariance() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return P_;
}

} // namespace rtklib_localization
