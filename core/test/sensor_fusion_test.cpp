#include <gtest/gtest.h>
#include "sensor_fusion.hpp"
#include <Eigen/Dense>

using namespace rtklib_localization;

class SensorFusionTest : public ::testing::Test {
protected:
    SensorFusionEKF ekf;

    void SetUp() override {
        // Initial state at origin, zero velocity
        Eigen::VectorXd initial_x = Eigen::VectorXd::Zero(15);
        Eigen::MatrixXd initial_P = Eigen::MatrixXd::Identity(15, 15) * 0.1;
        ekf.initialize(initial_x, initial_P);
    }
};

TEST_F(SensorFusionTest, Initialization) {
    Eigen::VectorXd state = ekf.getState();
    EXPECT_EQ(state.size(), 15);
    for (int i = 0; i < 15; ++i) {
        EXPECT_DOUBLE_EQ(state(i), 0.0);
    }
}

TEST_F(SensorFusionTest, PredictionConstantVelocity) {
    // Set initial velocity vx = 1.0
    Eigen::VectorXd x = Eigen::VectorXd::Zero(15);
    x(6) = 1.0;
    ekf.initialize(x, Eigen::MatrixXd::Identity(15, 15));

    // Predict for 1.0 second
    ekf.predict(1.0);

    Eigen::VectorXd state = ekf.getState();
    // Position x should be 1.0
    EXPECT_NEAR(state(0), 1.0, 1e-6);
    // Velocity vx should still be 1.0
    EXPECT_NEAR(state(6), 1.0, 1e-6);
}

TEST_F(SensorFusionTest, GNSSUpdate) {
    // Current position (0,0,0)
    // Update with GNSS at (10, 20, 30)
    MeasurementGNSS gnss;
    gnss.timestamp = 1.0;
    gnss.pos_enu = Eigen::Vector3d(10.0, 20.0, 30.0);
    gnss.covariance = Eigen::Matrix3d::Identity() * 0.01;

    ekf.updateGNSS(gnss);

    Eigen::VectorXd state = ekf.getState();
    // Should be close to the GNSS measurement
    EXPECT_NEAR(state(0), 10.0, 0.5);
    EXPECT_NEAR(state(1), 20.0, 0.5);
    EXPECT_NEAR(state(2), 30.0, 0.5);
}

TEST_F(SensorFusionTest, IMUUpdateVelocity) {
    // Initialize at rest
    // Push IMU with linear acceleration ax = 1.0 for 1 second
    // Note: Our EKF updateIMU currently treats accel as a direct measurement of state[12:15]
    // which represents acceleration in the state vector.

    MeasurementIMU imu;
    imu.timestamp = 1.0;
    imu.lin_accel = Eigen::Vector3d(1.0, 0.0, 0.0);
    imu.ang_vel = Eigen::Vector3d(0.0, 0.0, 0.0);

    ekf.updateIMU(imu);

    Eigen::VectorXd state = ekf.getState();
    // Acceleration state should be updated
    EXPECT_NEAR(state(12), 1.0, 0.1);

    // Predict to see if velocity increases
    ekf.predict(2.0); // dt = 1.0
    state = ekf.getState();

    // v = v0 + a*dt = 0 + 1*1 = 1
    EXPECT_NEAR(state(6), 1.0, 0.1);
    // x = x0 + v0*dt + 0.5*a*dt^2 = 0 + 0 + 0.5*1*1^2 = 0.5
    EXPECT_NEAR(state(0), 0.5, 0.1);
}

TEST_F(SensorFusionTest, FullTrajectoryFusion) {
    // 1. T=0.0: Initial GNSS at origin
    MeasurementGNSS gnss0;
    gnss0.timestamp = 0.0;
    gnss0.pos_enu = Eigen::Vector3d(0.0, 0.0, 0.0);
    gnss0.covariance = Eigen::Matrix3d::Identity() * 0.01;
    ekf.updateGNSS(gnss0);

    // 2. T=0.1 to T=1.0: Constant acceleration ax = 2.0 m/s^2
    for (int i = 1; i <= 10; ++i) {
        MeasurementIMU imu;
        imu.timestamp = i * 0.1;
        imu.lin_accel = Eigen::Vector3d(2.0, 0.0, 0.0);
        imu.ang_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
        ekf.updateIMU(imu);
    }

    // After 1.0s of 2.0 m/s^2 acceleration:
    // v = v0 + a*t = 0 + 2*1 = 2.0 m/s
    // p = p0 + v0*t + 0.5*a*t^2 = 0 + 0 + 0.5*2*1^2 = 1.0 m

    Eigen::VectorXd state = ekf.getState();
    EXPECT_NEAR(state(0), 1.0, 0.2);
    EXPECT_NEAR(state(6), 2.0, 0.2);

    // 3. T=1.1: GNSS update at (1.21, 0, 0)
    // Theoretical p at 1.1s: 0.5 * 2 * (1.1^2) = 1.21
    MeasurementGNSS gnss1;
    gnss1.timestamp = 1.1;
    gnss1.pos_enu = Eigen::Vector3d(1.21, 0.0, 0.0);
    gnss1.covariance = Eigen::Matrix3d::Identity() * 0.01;
    ekf.updateGNSS(gnss1);

    state = ekf.getState();
    // Final Position check
    EXPECT_NEAR(state(0), 1.21, 0.05);
    // Velocity should be around 2.2
    EXPECT_NEAR(state(6), 2.2, 0.1);
}
