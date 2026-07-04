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
    for (int i = 0; i < 15; ++i) {
        EXPECT_DOUBLE_EQ(state(i), 0.0);
    }
}

TEST_F(SensorFusionTest, PredictionConstantVelocity) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(15);
    x(6) = 1.0; // vx = 1.0
    ekf.initialize(x, Eigen::MatrixXd::Identity(15, 15));

    // Predict for 1.0 second with zero IMU input (except gravity compensation)
    // IMU input [0, 0, 9.81] means zero net acceleration in ENU
    ekf.predict(1.0, Eigen::Vector3d(0, 0, 9.81), Eigen::Vector3d::Zero());

    Eigen::VectorXd state = ekf.getState();
    EXPECT_NEAR(state(0), 1.0, 1e-6); // x = x0 + v*t
}

TEST_F(SensorFusionTest, IMUIntegration) {
    // Initialize at origin
    MeasurementGNSS gnss;
    gnss.timestamp = 0.0;
    gnss.pos_enu = Eigen::Vector3d(0, 0, 0);
    gnss.covariance = Eigen::Matrix3d::Identity();
    ekf.updateGNSS(gnss);

    // Fake alignment
    MeasurementIMU imu_align;
    imu_align.timestamp = 0.01;
    imu_align.lin_accel = Eigen::Vector3d(0, 0, 9.81);
    imu_align.ang_vel = Eigen::Vector3d(0, 0, 0);
    for(int i=0; i<20; ++i) ekf.updateIMU(imu_align);

    // Constant acceleration 1.0 m/s^2 for 1.0s
    // Input should be [1.0, 0, 9.81] to result in +1.0 ax after gravity subtraction
    for (int i = 1; i <= 10; ++i) {
        MeasurementIMU imu;
        imu.timestamp = i * 0.1;
        imu.lin_accel = Eigen::Vector3d(1.0, 0.0, 9.81);
        imu.ang_vel = Eigen::Vector3d(0.0, 0.0, 0.0);
        ekf.updateIMU(imu);
    }

    Eigen::VectorXd state = ekf.getState();
    // v = v0 + a*t = 0 + 1*1 = 1.0
    EXPECT_NEAR(state(6), 1.0, 0.1);
    // p = p0 + v0*t + 0.5*a*t^2 = 0 + 0 + 0.5*1*1^2 = 0.5
    EXPECT_NEAR(state(0), 0.5, 0.1);
}
