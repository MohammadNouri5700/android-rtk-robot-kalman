#include "robot_localization_ekf.hpp"
#include <cmath>
#include <algorithm>

namespace rtklib_localization {

    static double normalize_angle(double angle) {
        const double res = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (res < 0) return res + M_PI;
        return res - M_PI;
    }

    RobotLocalizationEKF::RobotLocalizationEKF() {
        x_.resize(STATE_SIZE);
        x_.setZero();

        P_.resize(STATE_SIZE, STATE_SIZE);
        P_.setIdentity();
        P_ *= 1e-9;

        Q_.resize(STATE_SIZE, STATE_SIZE);
        Q_.setIdentity();

        // FIX: Increased process noise for positional, velocity, and accel states.
        // This allows the GPS to gently correct the IMU drift instead of being overridden.
// In RobotLocalizationEKF::RobotLocalizationEKF()

// DECREASE the noise for Vx, Vy, Vz to mathematically enforce physical inertia.
// INCREASE the noise for Ax, Ay, Az because Android linear accelerometers are noisy.
        Q_.diagonal() <<
                      1e-1, 1e-1, 1e-1,   // X, Y, Z : (Loose) Let the RTK GPS dictate absolute position.
                1e-3, 1e-3, 1e-3,   // Roll, Pitch, Yaw : (Tight) Android's Rotation Vector is already highly filtered.
                1.0,  1.0,  1.0,    // Vx, Vy, Vz : (VERY LOOSE) Cars brake/accelerate fast. Force the EKF to trust GNSS Doppler velocity over IMU integration.
                1e-2, 1e-2, 1e-2,   // Vroll, Vpitch, Vyaw : (Moderate) Gyros are decently stable.
                2.0,  2.0,  2.0;    // Ax, Ay, Az : (MAX LOOSE) Phone accelerometers are garbage for dead-reckoning. Assume acceleration is highly erratic so the EKF filters out the bounce.
    }

    void RobotLocalizationEKF::initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_cov) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        x_ = initial_state;
        P_ = initial_cov;
        is_initialized_ = true;
    }

    void RobotLocalizationEKF::predict(double dt) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!is_initialized_) return;

        double roll = x_(Roll);
        double pitch = x_(Pitch);
        double yaw = x_(Yaw);
        double vx = x_(Vx);
        double vy = x_(Vy);
        double vz = x_(Vz);
        double vroll = x_(Vroll);
        double vpitch = x_(Vpitch);
        double vyaw = x_(Vyaw);
        double ax_body = x_(Ax);
        double ay_body = x_(Ay);
        double az_body = x_(Az);

        // 1. Convert Body-Frame Acceleration to Global ENU Frame
        // Rotation Matrix (Z-Y-X convention)
        double cr = std::cos(roll);  double sr = std::sin(roll);
        double cp = std::cos(pitch); double sp = std::sin(pitch);
        double cy = std::cos(yaw);   double sy = std::sin(yaw);

        // a_world = R * a_body
        double ax_world = (cp * cy) * ax_body + (sr * sp * cy - cr * sy) * ay_body + (cr * sp * cy + sr * sy) * az_body;
        double ay_world = (cp * sy) * ax_body + (sr * sp * sy + cr * cy) * ay_body + (cr * sp * sy - sr * cy) * az_body;
        double az_world = (-sp) * ax_body + (sr * cp) * ay_body + (cr * cp) * az_body;

        // 2. State Prediction (Kinematic Model)
        // Position integration using world-frame velocity and acceleration
        x_(X) += vx * dt + 0.5 * ax_world * dt * dt;
        x_(Y) += vy * dt + 0.5 * ay_world * dt * dt;
        x_(Z) += vz * dt + 0.5 * az_world * dt * dt;

        // Orientation integration
        x_(Roll) += vroll * dt;
        x_(Pitch) += vpitch * dt;
        x_(Yaw) += vyaw * dt;

        // Velocity integration using world-frame acceleration
        x_(Vx) += ax_world * dt;
        x_(Vy) += ay_world * dt;
        x_(Vz) += az_world * dt;

        wrapAngles();

        // 3. Jacobian F (Partial derivatives of state update)
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
        F(X, Vx) = dt; F(X, Ax) = 0.5 * dt * dt;
        F(Y, Vy) = dt; F(Y, Ay) = 0.5 * dt * dt;
        F(Z, Vz) = dt; F(Z, Az) = 0.5 * dt * dt;
        F(Roll, Vroll) = dt;
        F(Pitch, Vpitch) = dt;
        F(Yaw, Vyaw) = dt;
        F(Vx, Ax) = dt;
        F(Vy, Ay) = dt;
        F(Vz, Az) = dt;

        // 4. Covariance Prediction: P = FPF' + Q
        P_ = F * P_ * F.transpose() + Q_ * dt;
    }

    bool RobotLocalizationEKF::correct(const Eigen::VectorXd& measurement,
                                       const std::vector<int>& measurement_indices,
                                       const Eigen::MatrixXd& covariance,
                                       double mahalanobis_threshold) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!is_initialized_) return false;

        size_t m = measurement_indices.size();
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, STATE_SIZE);
        Eigen::VectorXd z(m);
        Eigen::VectorXd x_sub(m);

        for (size_t i = 0; i < m; ++i) {
            int idx = measurement_indices[i];
            H(i, idx) = 1.0;
            z(i) = measurement(i);
            x_sub(i) = x_(idx);
        }

        Eigen::VectorXd y = z - x_sub; // Innovation

        // Wrap angles in innovation
        for (size_t i = 0; i < m; ++i) {
            int idx = measurement_indices[i];
            if (idx == Roll || idx == Pitch || idx == Yaw) {
                y(i) = normalize_angle(y(i));
            }
        }

        Eigen::MatrixXd S = H * P_ * H.transpose() + covariance;
        Eigen::MatrixXd Si = S.inverse();

        // Mahalanobis Gating
        if (mahalanobis_threshold > 0.0) {
            double d2 = y.transpose() * Si * y;
            if (d2 > mahalanobis_threshold * mahalanobis_threshold) {
                return false;
            }
        }

        Eigen::MatrixXd K = P_ * H.transpose() * Si;

        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * P_;

        wrapAngles();
        return true;
    }

    void RobotLocalizationEKF::wrapAngles() {
        x_(Roll) = normalize_angle(x_(Roll));
        x_(Pitch) = normalize_angle(x_(Pitch));
        x_(Yaw) = normalize_angle(x_(Yaw));
    }

    Eigen::VectorXd RobotLocalizationEKF::getState() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return x_;
    }

    Eigen::MatrixXd RobotLocalizationEKF::getCovariance() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return P_;
    }

} // namespace rtklib_localization