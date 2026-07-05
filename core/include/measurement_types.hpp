
#pragma once
#include <Eigen/Dense>
#include <variant>
#include <vector>

namespace rtklib_localization {

struct MeasurementIMU {
    double timestamp;
    Eigen::Vector3d lin_accel;
    Eigen::Vector3d ang_vel;
};

struct MeasurementGNSS {
    double timestamp;
    Eigen::Vector3d pos_enu; // WGS84 or ENU based on usage, but we'll use it for corrections
    Eigen::Vector3d vel_enu;
    bool has_velocity;
    Eigen::Matrix3d covariance;
};

struct MeasurementOrientation {
    double timestamp;
    double roll;
    double pitch;
    double yaw;
};

using Measurement = std::variant<MeasurementIMU, MeasurementGNSS, MeasurementOrientation>;

struct StateSnapshot {
    double timestamp;
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
};

} // namespace rtklib_localization
