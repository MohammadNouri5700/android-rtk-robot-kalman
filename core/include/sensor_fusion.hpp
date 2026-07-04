
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
    Eigen::Vector3d vel_enu; // Velocity in ENU frame [vEast, vNorth, vUp]
    bool has_velocity{false};
    Eigen::Matrix3d covariance;
};

class SensorFusionEKF {
public:
    SensorFusionEKF();
    ~SensorFusionEKF() = default;

    void initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov);
    void predict(double current_time, const Eigen::Vector3d& linear_accel, const Eigen::Vector3d& angular_vel);
    void updateIMU(const MeasurementIMU& imu);
    void updateGNSS(const MeasurementGNSS& gnss);
    
    bool isAligned() const { return is_aligned_; }

    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

    // Coordinate Conversion Utilities (Exposed for JNI)
    Eigen::Vector3d wgs84ToEnu(double lat, double lon, double alt) const;
    Eigen::Vector3d enuToWgs84(const Eigen::Vector3d& enu) const;

private:
    void initializeInternal(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov);
    void predictInternal(double current_time, const Eigen::Vector3d& linear_accel, const Eigen::Vector3d& angular_vel);
    void integrateStateInternal(double dt, const Eigen::Vector3d& linear_accel, const Eigen::Vector3d& angular_vel);
    Eigen::Matrix3d skew(const Eigen::Vector3d& v) const;

    mutable std::mutex state_mutex_;
    bool is_initialized_{false};
    bool is_aligned_{false};
    bool has_origin_{false};
    double last_update_time_{0.0};
    int consecutive_gnss_rejections_{0};

    // Store last IMU for prediction between GNSS
    Eigen::Vector3d last_accel_{0,0,0};
    Eigen::Vector3d last_gyro_{0,0,0};

    // Alignment logic
    int alignment_samples_{0};
    Eigen::Vector3d accel_sum_{0,0,0};
    Eigen::Vector3d gyro_sum_{0,0,0};
    static constexpr int ALIGNMENT_THRESHOLD = 20;

    // Reference Origin for ENU (Latitude, Longitude in radians, Altitude in meters)
    double origin_lat_{0.0};
    double origin_lon_{0.0};
    double origin_alt_{0.0};

    // State Vector (15x1): [pos_enu(3), ori_euler(3), vel_enu(3), gyro_bias(3), accel_bias(3)]
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_; // State Covariance
    Eigen::VectorXd Q_diag_; // Process Noise Covariance Diagonals
};

} // namespace rtklib_localization
