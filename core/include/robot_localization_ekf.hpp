
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <mutex>

namespace rtklib_localization {

/**
 * @brief Standalone port of robot_localization EKF logic.
 * Optimized for Android/Embedded usage without ROS dependencies.
 */
class RobotLocalizationEKF {
public:
    enum StateMembers {
        X = 0, Y, Z,
        Roll, Pitch, Yaw,
        Vx, Vy, Vz,
        Vroll, Vpitch, Vyaw,
        Ax, Ay, Az,
        STATE_SIZE = 15
    };

    RobotLocalizationEKF();
    ~RobotLocalizationEKF() = default;

    /**
     * @brief Initialize the filter with a starting state and covariance.
     */
    void initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov);

    /**
     * @brief Predict the state forward by dt seconds.
     */
    void predict(double dt);

    /**
     * @brief Correct the state using a measurement.
     * @param measurement The measurement vector (size varies).
     * @param measurement_indices Indices of the state vector being measured.
     * @param covariance The measurement covariance.
     * @param mahalanobis_threshold Threshold for outlier rejection (0 to disable).
     * @return true if the measurement was accepted.
     */
    bool correct(const Eigen::VectorXd& measurement,
                 const std::vector<int>& measurement_indices,
                 const Eigen::MatrixXd& covariance,
                 double mahalanobis_threshold = 0.0);

    // Getters
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;
    bool isInitialized() const { return is_initialized_; }

private:
    void wrapAngles();

    Eigen::VectorXd x_; // State vector (15x1)
    Eigen::MatrixXd P_; // State covariance (15x15)
    Eigen::MatrixXd Q_; // Process noise covariance (15x15)

    bool is_initialized_{false};
    mutable std::mutex state_mutex_;
};

} // namespace rtklib_localization
