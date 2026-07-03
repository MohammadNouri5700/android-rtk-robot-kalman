# Walkthrough: Unit Tests for Math Verification

I have implemented a comprehensive unit testing suite to verify the mathematical correctness of the Extended Kalman Filter (EKF) used for sensor fusion.

## Changes Made

### 1. GoogleTest Integration
Updated the [CMakeLists.txt](file:///Users/leo/AndroidStudioProjects/android-rtk-robot-kalman/android/app/src/main/cpp/CMakeLists.txt) to pull in **GoogleTest** via `FetchContent`. This allows us to write native C++ tests that run directly against the core math logic without needing the Android framework.

### 2. C++ Math Tests
Created [sensor_fusion_test.cpp](file:///Users/leo/AndroidStudioProjects/android-rtk-robot-kalman/core/test/sensor_fusion_test.cpp) which performs the following checks:
- **Initialization**: Ensures the state vector and covariance matrix are correctly zeroed and sized.
- **State Prediction**: Verifies that the kinematic model correctly predicts position based on velocity over time ($x = x_0 + v \cdot dt$).
- **GNSS Fusion**: Confirms that adding a GNSS measurement correctly pulls the state towards the observed position.
- **IMU Fusion**: Tests that linear acceleration correctly updates the velocity and position states during prediction.

### 3. Kotlin Integration Tests
Added [LocalizationEngineTest.kt](file:///Users/leo/AndroidStudioProjects/android-rtk-robot-kalman/android/app/src/test/java/com/example/rtklib_localization/LocalizationEngineTest.kt) to verify the data classes and basic engine state mapping on the Android side.

## Verification Results

### Automated Tests
- **Kotlin Unit Tests**: PASSED
  - Ran `./gradlew :app:testDebugUnitTest`
  - Result: `2 passed`
- **Native Build**: SUCCESS
  - Verified that the `sensor_fusion_test` target compiles correctly with GoogleTest and Eigen3.

### How to Run Native Tests on Device
To run the native math tests on an Android device/emulator:
1. Build the test executable: `./gradlew app:assembleDebug`
2. Push the executable to the device: `adb push app/build/intermediates/cmake/debug/obj/arm64-v8a/sensor_fusion_test /data/local/tmp/`
3. Run the tests: `adb shell /data/local/tmp/sensor_fusion_test`

> [!TIP]
> The native tests are the most critical for verifying the EKF math, as they test the Eigen-based calculations directly.
