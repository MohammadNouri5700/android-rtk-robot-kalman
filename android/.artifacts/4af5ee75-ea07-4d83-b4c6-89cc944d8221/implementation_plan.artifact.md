# Implementation Plan: Phase 4 Fusion Verification

Extend the existing test suite to explicitly verify the "Phase 4" roadmap: the end-to-end fusion of GNSS and IMU data to produce a precise final position.

## User Review Required

> [!NOTE]
> This plan adds a complex "Mission Simulation" test that verifies the EKF's ability to handle drift and corrections over a 10-second window.

## Proposed Changes

### [Core C++ Component]

#### [MODIFY] [sensor_fusion_test.cpp](file:///Users/leo/AndroidStudioProjects/android-rtk-robot-kalman/core/test/sensor_fusion_test.cpp)
- **Add `FusedTrajectoryTest`**:
    1. Simulate a robot moving at 1m/s.
    2. Feed high-frequency IMU data (100Hz) with a slight bias to simulate drift.
    3. Feed low-frequency GNSS data (1Hz) with high precision.
    4. Verify that the "Final Position" stays within a tight bound (e.g., 5cm) of the ground truth, proving the GNSS correction is working.

## Verification Plan

### Automated Tests
- Run the updated native tests:
  - `adb shell /data/local/tmp/sensor_fusion_test` (on device)
  - Or via host-side build if configured.

### Manual Verification
- Check the test output for "Innovation" and "Covariance" reduction logs to ensure the EKF is numerically stable during the long-running simulation.
