#include <gtest/gtest.h>
#include "sensor_fusion.hpp"
#include <Eigen/Dense>

using namespace rtklib_localization;

class SensorFusionTest : public ::testing::Test {
protected:
    SensorFusionEKF ekf;

    void SetUp() override {
        Eigen::VectorXd initial_x = Eigen::VectorXd::Zero(15);
        Eigen::MatrixXd initial_P = Eigen::MatrixXd::Identity(15, 15) * 0.1;
        ekf.initialize(initial_x, initial_P);
    }
};

TEST_F(SensorFusionTest, Initialization) {
    Eigen::VectorXd state = ekf.getState();
    EXPECT_EQ(state.size(), 15);
}

TEST_F(SensorFusionTest, PredictionConstantVelocity) {
    // We need to establish an origin first for ENU logic
    MeasurementGNSS gnss;
    gnss.timestamp = 0.0;
    gnss.pos_enu = Eigen::Vector3d(0, 0, 0); // Origin at (0,0,0)
    gnss.covariance = Eigen::Matrix3d::Identity();
    ekf.updateGNSS(gnss);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(15);
    x(6) = 1.0; // vx = 1.0
    ekf.initialize(x, Eigen::MatrixXd::Identity(15, 15));

    // To trigger prediction, we push an IMU point
    MeasurementIMU imu;
    imu.timestamp = 1.0;
    imu.lin_accel = Eigen::Vector3d(0, 0, 9.81);
    imu.ang_vel = Eigen::Vector3d(0, 0, 0);
    ekf.updateIMU(imu);

    Eigen::VectorXd state = ekf.getState();
    EXPECT_NEAR(state(0), 1.0, 0.1); // x = x0 + v*t
}

TEST_F(SensorFusionTest, IMUIntegration) {
    // Initialize at origin
    MeasurementGNSS gnss;
    gnss.timestamp = 0.0;
    gnss.pos_enu = Eigen::Vector3d(0, 0, 0);
    gnss.covariance = Eigen::Matrix3d::Identity();
    ekf.updateGNSS(gnss);

    // Constant acceleration 1.0 m/s^2 for 1.0s
    for (int i = 1; i <= 10; ++i) {
        MeasurementIMU imu;
        imu.timestamp = i * 0.1;
        imu.lin_accel = Eigen::Vector3d(1.0, 0.0, 9.81);
        imu.ang_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
        ekf.updateIMU(imu);
    }

    Eigen::VectorXd state = ekf.getState();
    // v = v0 + a*t = 0 + 1*1 = 1.0
    EXPECT_NEAR(state(6), 1.0, 0.2);
    // p = p0 + v0*t + 0.5*a*t^2 = 0 + 0 + 0.5*1*1^2 = 0.5
    EXPECT_NEAR(state(0), 0.5, 0.2);
}
