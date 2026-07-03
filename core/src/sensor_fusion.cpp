#include "sensor_fusion.hpp"
#include <cmath>

namespace rtklib_localization {

SensorFusionEKF::SensorFusionEKF() {
    x_.resize(15);
    x_.setZero();
    
    P_.resize(15, 15);
    P_.setIdentity();
    
    Q_.resize(15, 15);
    Q_.setIdentity() * 0.05;
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
    
    if (!is_initialized_) {
        Eigen::VectorXd init_x = Eigen::VectorXd::Zero(15);
        init_x.segment<3>(0) = gnss.pos_enu;
        // Safe to call now because it's the Internal version (no lock attempt)
        initializeInternal(init_x, Eigen::MatrixXd::Identity(15, 15) * 1.0);
        last_update_time_ = gnss.timestamp;
        return;
    }

    predictInternal(gnss.timestamp);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Vector3d z = gnss.pos_enu;
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

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 15);
    H.block<3, 3>(0, 9) = Eigen::MatrixXd::Identity(3, 3);
    H.block<3, 3>(3, 12) = Eigen::MatrixXd::Identity(3, 3);

    Eigen::Matrix<double, 6, 1> z;
    z << imu.ang_vel, imu.lin_accel;

    Eigen::Matrix<double, 6, 1> y = z - H * x_;
    
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6) * 0.02;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
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
