# Android RTK Robot Kalman

[![JitPack](https://jitpack.io/v/MohammadNouri5700/android-rtk-robot-kalman.svg)](https://jitpack.io/#MohammadNouri5700/android-rtk-robot-kalman)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![API](https://img.shields.io/badge/API-26%2B-brightgreen.svg?style=flat)](https://android-arsenal.com/api?level=26)
[![Android](https://img.shields.io/badge/Platform-Android-brightgreen.svg)](https://developer.android.com)
[![Kotlin](https://img.shields.io/badge/Language-Kotlin-blue.svg)](https://kotlinlang.org)
[![C++](https://img.shields.io/badge/Language-C%2B%2B20-orange.svg)](https://isocpp.org)
[![Eigen3](https://img.shields.io/badge/Dependency-Eigen3-lightgrey.svg)](https://eigen.tuxfamily.org)

**Android RTK Robot Kalman** is a high-performance, enterprise-grade sensor fusion library specifically engineered for robotics and autonomous navigation on Android. It leverages an **Extended Kalman Filter (EKF)** to fuse high-precision RTK GNSS data with high-frequency IMU measurements, providing a robust, low-latency 15-dimensional state estimate.

---

## Overview

In demanding environments (urban canyons, agricultural fields, construction sites), raw GPS data is often noisy or suffers from multipath errors. This library provides:
- **Multipath Mitigation**: Uses Mahalanobis distance-based outlier rejection to discard erratic GNSS jumps.
- **Trajectory Smoothing**: Integrates IMU data at high frequencies (100Hz+) to fill gaps between GNSS updates (1Hz - 10Hz).
- **Full Kinematic State**: Estimates not just position, but also orientation (Euler angles), linear/angular velocity, and linear acceleration.
- **Embedded Optimization**: Core logic is implemented in C++20 using the **Eigen3** matrix library for maximum throughput and deterministic execution.

---

## How It Works (The Math)

The library implements a standard **15-state Extended Kalman Filter** designed for kinematic motion.

### 1. The State Vector
The filter tracks the following state vector $x \in \mathbb{R}^{15}$:
$$x = [p_x, p_y, p_z, \phi, \theta, \psi, v_x, v_y, v_z, \dot{\phi}, \dot{\theta}, \dot{\psi}, a_x, a_y, a_z]^T$$
*   **Position**: ENU (East-North-Up) coordinates.
*   **Orientation**: Roll, Pitch, Yaw (Euler angles).
*   **Velocity**: Linear and Angular velocities.
*   **Acceleration**: Linear acceleration in the body frame.

### 2. Prediction Step (Kinematic Model)
When new IMU data arrives, the filter predicts the next state using a 3D kinematic model. It performs **Body-to-World frame transformation** for acceleration:
$$a_{world} = R(\phi, \theta, \psi) \cdot a_{body}$$
The state is then propagated forward:
$$x_{k|k-1} = f(x_{k-1}, \Delta t)$$
The error covariance $P$ is updated via the Jacobian $F$:
$$P_{k|k-1} = F P_{k-1} F^T + Q \Delta t$$

### 3. Measurement Update
When absolute sensors (GNSS, Magnetometers) provide data, the filter corrects its prediction:
- **Innovation (y)**: $y = z - Hx$
- **Innovation Covariance (S)**: $S = HPH^T + R$
- **Kalman Gain (K)**: $K = P H^T S^{-1}$
- **State Update**: $x_{k} = x_{k-1} + Ky$

---

## Key Features

- **Native Performance**: Zero garbage collection overhead during core math execution.
- **Zero-Dependency Core**: The C++ EKF engine is self-contained and does not require ROS or external localization frameworks.
- **Real-Time Processing**: Designed for high-frequency sensor streams (up to 400Hz IMU).
- **History & Rollback**: Handles late-arriving measurements by maintaining a sliding window of previous states.
- **RTK Integrated**: Specialized handling for RTKLIB-based raw observations and fixed/float GNSS solutions.

---

## Installation

### 1. Add JitPack to `settings.gradle.kts`
```kotlin
dependencyResolutionManagement {
    repositories {
        mavenCentral()
        maven { url = uri("https://jitpack.io") }
    }
}
```

### 2. Add Dependency to `build.gradle.kts`
```kotlin
dependencies {
    implementation("com.github.MohammadNouri5700:android-rtk-robot-kalman:1.0.7")
}
```

---

## Quick Start

### Basic Implementation
Initialize the engine and observe the fused state using modern Kotlin Coroutines.

```kotlin
val rtkEngine = RtkRobotEngine()

// Collect fused state updates (StateFlow)
lifecycleScope.launch {
    rtkEngine.engineState.collect { state ->
        state?.let { 
            println("Fused Pos: ${it.latitude}, ${it.longitude} | Yaw: ${it.yaw}")
        }
    }
}

// Push IMU Data (e.g., from SensorEventListener)
rtkEngine.pushIMU(
    SystemClock.elapsedRealtimeNanos(),
    floatArrayOf(ax, ay, az), // Linear Acceleration
    floatArrayOf(wx, wy, wz)  // Angular Velocity
)

// Push GNSS Correction
rtkEngine.pushGNSS(
    SystemClock.elapsedRealtimeNanos(),
    lat, lon, alt, 
    ve, vn, vu, 
    hasVelocity = true,
    covariance = doubleArrayOf(...) // 3x3 Pos Covariance
)
```

---

## API Reference

| Method | Description |
| :--- | :--- |
| `pushIMU(...)` | Pushes accelerometer and gyroscope data into the prediction step. |
| `pushGNSS(...)` | Corrects the state using absolute latitude/longitude/altitude. |
| `pushOrientation(...)` | Corrects the orientation using external AHRS or Magnetometer. |
| `pushRawObservations(...)` | For advanced RTK integration using raw GNSS observations. |
| `engineState` | A `StateFlow<StateVector>` providing the current fused estimate. |
| `close()` | Releases native memory (Required: `AutoCloseable`). |

---

## Contributing

Contributions are welcome! If you find a bug or have a feature request, please open an issue. For code contributions, please follow these steps:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/amazing-feature`).
3. Commit your changes (`git commit -m 'Add amazing feature'`).
4. Push to the branch (`git push origin feature/amazing-feature`).
5. Open a Pull Request.

---

## License

Distributed under the **Apache License 2.0**. See `LICENSE` for more information.

---
**Maintained by Mohammad Nouri** - [GitHub](https://github.com/MohammadNouri5700)
