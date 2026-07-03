
#pragma once
#include <Eigen/Dense>
#include <mutex>

namespace rtklib_localization {

struct MeasurementIMU {
    double timestamp; // Epoch seconds
    Eigen::Vector3d lin_accel;
    Eigen::Vector3d ang_vel;
};

struct MeasurementGNSS {
    double timestamp;
    Eigen::Vector3d pos_enu; // Extracted from RTKLIB baseline
    Eigen::Matrix3d covariance;
};

class SensorFusionEKF {
public:
    SensorFusionEKF();
    ~SensorFusionEKF() = default;

    void initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov);
    void predict(double current_time);
    void updateIMU(const MeasurementIMU& imu);
    void updateGNSS(const MeasurementGNSS& gnss);
    
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

private:
    void initializeInternal(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov);
    void predictInternal(double current_time);
    void integrateStateInternal(double dt);

    mutable std::mutex state_mutex_;
    bool is_initialized_{false};
    double last_update_time_{0.0};

    // State Vector: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz, ax, ay, az]
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_; // State Covariance
    Eigen::MatrixXd Q_; // Process Noise Covariance
};

} // namespace rtklib_localization
