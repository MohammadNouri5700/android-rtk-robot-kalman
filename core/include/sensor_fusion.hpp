
#pragma once
#include "robot_localization_ekf.hpp"
#include "rtklib_interface.hpp"
#include "measurement_types.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <deque>
#include <map>

namespace rtklib_localization {

class SensorFusionEKF {
public:
    SensorFusionEKF();
    ~SensorFusionEKF() = default;

    void initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov);
    void updateIMU(const MeasurementIMU& imu);
    void updateGNSS(const MeasurementGNSS& gnss);
    void updateOrientation(const MeasurementOrientation& orient);
    void updateRawObservations(double timestamp, const std::vector<std::vector<double>>& raw_data);

    bool isAligned() const { return is_aligned_; }

    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

    // Coordinate Conversion Utilities
    Eigen::Vector3d wgs84ToEnu(double lat, double lon, double alt) const;
    Eigen::Vector3d enuToWgs84(const Eigen::Vector3d& enu) const;

private:
    void initializeInternal(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov);
    void processQueue();
    void processMeasurement(const Measurement& m);
    void saveState(double timestamp);
    void rollbackTo(double timestamp);

    mutable std::mutex state_mutex_;
    RobotLocalizationEKF ekf_;
    RTKEngine rtk_engine_;

    std::deque<Measurement> measurement_queue_;
    std::map<double, Measurement> measurement_history_;
    std::map<double, StateSnapshot> state_history_;
    static constexpr size_t MAX_HISTORY_SIZE = 100;

    bool is_initialized_{false};
    bool is_aligned_{false};
    bool has_origin_{false};
    double last_update_time_{0.0};
    int consecutive_gnss_rejections_{0};
    int static_samples_{0};

    // Store last IMU for prediction between GNSS
    Eigen::Vector3d last_accel_{0,0,0};
    Eigen::Vector3d last_gyro_{0,0,0};

    // Reference Origin for ENU
    double origin_lat_{0.0};
    double origin_lon_{0.0};
    double origin_alt_{0.0};
};

} // namespace rtklib_localization
