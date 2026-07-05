#include "sensor_fusion.hpp"
#include "rtklib.h"
#include <cmath>
#include <Eigen/Geometry>
#include <android/log.h>
#include <algorithm>

#define TAG "RTK_Native"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

namespace rtklib_localization {

    SensorFusionEKF::SensorFusionEKF() {
        last_update_time_ = 0.0;
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
        ekf_.initialize(initial_state, initial_cov);
        is_initialized_ = true;
        consecutive_gnss_rejections_ = 0;
        state_history_.clear();
        measurement_history_.clear();
    }

    void SensorFusionEKF::updateGNSS(const MeasurementGNSS& gnss) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        measurement_queue_.push_back(gnss);
        processQueue();
    }

    void SensorFusionEKF::updateRawObservations(double timestamp, const std::vector<std::vector<double>>& raw_data) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (rtk_engine_.processRaw(timestamp, raw_data)) {
            sol_t sol = rtk_engine_.getSolution();

            double pos[3], vel[3];
            ecef2pos(sol.rr, pos);

            MeasurementGNSS gnss;
            gnss.timestamp = timestamp;
            gnss.pos_enu = Eigen::Vector3d(pos[0] * R2D, pos[1] * R2D, pos[2]);
            gnss.has_velocity = true;

            double vel_ecef[3] = {sol.rr[3], sol.rr[4], sol.rr[5]};
            double vel_enu[3];
            ecef2enu(pos, vel_ecef, vel_enu);
            gnss.vel_enu = Eigen::Vector3d(vel_enu[0], vel_enu[1], vel_enu[2]);

            gnss.covariance << sol.qr[0], sol.qr[3], sol.qr[5],
                    sol.qr[3], sol.qr[1], sol.qr[4],
                    sol.qr[5], sol.qr[4], sol.qr[2];

            measurement_queue_.push_back(gnss);
            processQueue();
        }
    }

    void SensorFusionEKF::updateIMU(const MeasurementIMU& imu) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        measurement_queue_.push_back(imu);
        processQueue();
    }

    void SensorFusionEKF::updateOrientation(const MeasurementOrientation& orient) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        measurement_queue_.push_back(orient);
        processQueue();
    }

    void SensorFusionEKF::processQueue() {
        if (measurement_queue_.empty()) return;

        std::sort(measurement_queue_.begin(), measurement_queue_.end(), [](const Measurement& a, const Measurement& b) {
            auto get_ts = [](const Measurement& m) {
                return std::visit([](auto&& arg) { return arg.timestamp; }, m);
            };
            return get_ts(a) < get_ts(b);
        });

        while (!measurement_queue_.empty()) {
            Measurement m = measurement_queue_.front();
            double m_time = std::visit([](auto&& arg) { return arg.timestamp; }, m);

            if (is_initialized_ && m_time < last_update_time_) {
                rollbackTo(m_time);
            }

            processMeasurement(m);
            measurement_history_[m_time] = m;

            if (measurement_history_.size() > MAX_HISTORY_SIZE) {
                measurement_history_.erase(measurement_history_.begin());
            }

            measurement_queue_.pop_front();
        }
    }

    void SensorFusionEKF::processMeasurement(const Measurement& m) {
        double m_time = std::visit([](auto&& arg) { return arg.timestamp; }, m);

        if (std::holds_alternative<MeasurementIMU>(m)) {
            auto& imu = std::get<MeasurementIMU>(m);
            if (!is_initialized_) return;

            double dt = imu.timestamp - last_update_time_;
            if (dt > 0) ekf_.predict(dt);

            // 1. ZUPT (Zero-Velocity Update)
            if (imu.lin_accel.norm() < 0.6 && imu.ang_vel.norm() < 0.1) {
                if (++static_samples_ > 5) {
                    std::vector<int> indices = {RobotLocalizationEKF::Vx, RobotLocalizationEKF::Vy, RobotLocalizationEKF::Vz};
                    Eigen::VectorXd z = Eigen::VectorXd::Zero(3);
                    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 0.01;
                    ekf_.correct(z, indices, R);
                    static_samples_ = 5;
                }
            } else {
                static_samples_ = 0;
            }

            // 2. Active Acceleration Correction
            {
                std::vector<int> indices = {RobotLocalizationEKF::Ax, RobotLocalizationEKF::Ay, RobotLocalizationEKF::Az};
                Eigen::VectorXd z(3);
                z << imu.lin_accel.x(), imu.lin_accel.y(), imu.lin_accel.z();

                // De-weight noisy Android Linear Acceleration
                Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 2.5;
                ekf_.correct(z, indices, R);
            }

            // 3. Gyroscopic correction
            {
                std::vector<int> indices = {RobotLocalizationEKF::Vroll, RobotLocalizationEKF::Vpitch, RobotLocalizationEKF::Vyaw};
                Eigen::VectorXd z(3);
                z << imu.ang_vel.x(), imu.ang_vel.y(), imu.ang_vel.z();
                Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 0.001;
                ekf_.correct(z, indices, R);
            }

            last_update_time_ = imu.timestamp;
            saveState(imu.timestamp);

        } else if (std::holds_alternative<MeasurementGNSS>(m)) {
            auto& gnss = std::get<MeasurementGNSS>(m);

            if (!has_origin_) {
                if (gnss.covariance(0,0) < 2500.0) {
                    origin_lat_ = gnss.pos_enu.x() * D2R;
                    origin_lon_ = gnss.pos_enu.y() * D2R;
                    origin_alt_ = gnss.pos_enu.z();
                    has_origin_ = true;
                } else {
                    return;
                }
            }

            Eigen::Vector3d pos_enu = wgs84ToEnu(gnss.pos_enu.x(), gnss.pos_enu.y(), gnss.pos_enu.z());

            // ==========================================================
            // SMART 10-METER STICKY FILTER (Stationary Wandering Rejection)
            // ==========================================================
            if (is_initialized_) {
                Eigen::VectorXd current_state = ekf_.getState();
                double dx = pos_enu.x() - current_state(RobotLocalizationEKF::X);
                double dy = pos_enu.y() - current_state(RobotLocalizationEKF::Y);
                double distance_from_ekf = std::sqrt(dx * dx + dy * dy);

                // Calculate current EKF speed (magnitude of Vx, Vy)
                double current_speed = std::sqrt(
                    std::pow(current_state(RobotLocalizationEKF::Vx), 2) +
                    std::pow(current_state(RobotLocalizationEKF::Vy), 2)
                );

                // If speed is very low (< 1 m/s) OR the IMU says we are parked (static_samples_ > 5)
                if (current_speed < 1.0 || static_samples_ > 5) {
                    // And the GPS wandered less than 10 meters from our sticky position
                    if (distance_from_ekf < 10.0) {
                        // LOGD("Stationary noise ignored. Dist: %.2fm, Speed: %.2fm/s", distance_from_ekf, current_speed);

                        // Drop the noisy measurement to keep the marker perfectly still
                        // But update the time/state history so the EKF doesn't break
                        last_update_time_ = gnss.timestamp;
                        saveState(gnss.timestamp);
                        return;
                    }
                }
            }
            // ==========================================================

            // --- KINEMATIC CLAMP ---
            double speed_ms = gnss.vel_enu.norm();
            bool use_velocity = gnss.has_velocity;

            // If the reported GNSS speed is over ~160 km/h (45 m/s), reject velocity measurement
            if (use_velocity && speed_ms > 45.0) {
                LOGD("GNSS Velocity Outlier Detected: %f m/s. Rejecting velocity update.", speed_ms);
                use_velocity = false;
            }
            // -----------------------

            if (!is_initialized_) {
                Eigen::VectorXd init_x = Eigen::VectorXd::Zero(RobotLocalizationEKF::STATE_SIZE);
                init_x.segment<3>(RobotLocalizationEKF::X) = pos_enu;
                if (use_velocity) init_x.segment<3>(RobotLocalizationEKF::Vx) = gnss.vel_enu;

                Eigen::MatrixXd init_P = Eigen::MatrixXd::Identity(RobotLocalizationEKF::STATE_SIZE, RobotLocalizationEKF::STATE_SIZE) * 1.0;
                init_P.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 5.0;

                initializeInternal(init_x, init_P);
                last_update_time_ = gnss.timestamp;
                saveState(gnss.timestamp);
                return;
            }

            double dt = gnss.timestamp - last_update_time_;
            if (dt > 0) ekf_.predict(dt);

            std::vector<int> indices = {RobotLocalizationEKF::X, RobotLocalizationEKF::Y, RobotLocalizationEKF::Z};
            Eigen::VectorXd z(3);
            z << pos_enu.x(), pos_enu.y(), pos_enu.z();
            Eigen::MatrixXd R = gnss.covariance;

            if (use_velocity) {
                indices.push_back(RobotLocalizationEKF::Vx); indices.push_back(RobotLocalizationEKF::Vy); indices.push_back(RobotLocalizationEKF::Vz);
                Eigen::VectorXd z_new(6); z_new.segment<3>(0) = z; z_new.segment<3>(3) = gnss.vel_enu; z = z_new;

                Eigen::MatrixXd R_new = Eigen::MatrixXd::Zero(6, 6);
                R_new.block<3, 3>(0, 0) = R;
                // DE-WEIGHT GNSS VELOCITY MEASUREMENT: Increase R from 0.5 to 4.0
                R_new.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 4.0;
                R = R_new;
            }

            // Widen Mahalanobis gate specifically for mobile device variances
            double gate_threshold = is_aligned_ ? 10.0 : 15.0;
            bool accepted = ekf_.correct(z, indices, R, gate_threshold);

            if (!accepted) {
                consecutive_gnss_rejections_++;
                if (consecutive_gnss_rejections_ >= 5) {
                    Eigen::VectorXd state = ekf_.getState();
                    state.segment<3>(RobotLocalizationEKF::X) = pos_enu;

                    if (use_velocity) {
                        state.segment<3>(RobotLocalizationEKF::Vx) = gnss.vel_enu;
                    } else {
                        // Nullify velocity if forced into a hard reset to prevent runaway drift
                        state.segment<3>(RobotLocalizationEKF::Vx) = Eigen::Vector3d::Zero();
                    }

                    ekf_.initialize(state, ekf_.getCovariance() * 2.0);
                    consecutive_gnss_rejections_ = 0;
                }
            } else {
                consecutive_gnss_rejections_ = 0;
            }
            last_update_time_ = gnss.timestamp;
            saveState(gnss.timestamp);

        } else if (std::holds_alternative<MeasurementOrientation>(m)) {
            auto& orient = std::get<MeasurementOrientation>(m);
            if (is_initialized_) {
                Eigen::VectorXd state = ekf_.getState();
                state(RobotLocalizationEKF::Roll) = orient.roll;
                state(RobotLocalizationEKF::Pitch) = orient.pitch;
                state(RobotLocalizationEKF::Yaw) = orient.yaw;
                ekf_.initialize(state, ekf_.getCovariance());
                is_aligned_ = true;
                saveState(last_update_time_);
            }
        }
    }

    void SensorFusionEKF::saveState(double timestamp) {
        state_history_[timestamp] = {timestamp, ekf_.getState(), ekf_.getCovariance()};
        if (state_history_.size() > MAX_HISTORY_SIZE) {
            state_history_.erase(state_history_.begin());
        }
    }

    void SensorFusionEKF::rollbackTo(double timestamp) {
        auto it = state_history_.lower_bound(timestamp);
        if (it == state_history_.begin()) return;
        --it;

        ekf_.initialize(it->second.state, it->second.covariance);
        last_update_time_ = it->first;

        auto erase_it = state_history_.upper_bound(last_update_time_);
        state_history_.erase(erase_it, state_history_.end());

        auto m_it = measurement_history_.upper_bound(last_update_time_);
        while (m_it != measurement_history_.end()) {
            processMeasurement(m_it->second);
            ++m_it;
        }
    }

    Eigen::VectorXd SensorFusionEKF::getState() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        Eigen::VectorXd ekf_state = ekf_.getState();
        Eigen::VectorXd output = Eigen::VectorXd::Zero(15);
        output.segment<3>(0) = ekf_state.segment<3>(RobotLocalizationEKF::X);
        output.segment<3>(3) = ekf_state.segment<3>(RobotLocalizationEKF::Roll);
        output.segment<3>(6) = ekf_state.segment<3>(RobotLocalizationEKF::Vx);
        output.segment<3>(9) = ekf_state.segment<3>(RobotLocalizationEKF::Vroll);
        output.segment<3>(12) = ekf_state.segment<3>(RobotLocalizationEKF::Ax);
        return output;
    }

    Eigen::MatrixXd SensorFusionEKF::getCovariance() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return ekf_.getCovariance();
    }

} // namespace rtklib_localization