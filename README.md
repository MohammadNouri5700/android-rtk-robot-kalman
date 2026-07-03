# Android RTK Robot Kalman

[![JitPack](https://jitpack.io/v/MohammadNouri5700/android-rtk-robot-kalman.svg)](https://jitpack.io/#MohammadNouri5700/android-rtk-robot-kalman)

A high-performance, enterprise-tier Android SDK for robotics and autonomous systems. This library provides advanced sensor fusion by combining high-precision RTK GNSS positioning with high-frequency IMU data using an Extended Kalman Filter (EKF).

Powered by a native C++20 engine, Eigen3 for lightning-fast matrix operations, and a modern, lifecycle-aware Kotlin API.

---

## Key Features

- **True Sensor Fusion**: Fuses asynchronous GNSS (Global Navigation Satellite System) and IMU (Inertial Measurement Unit) data into a single, reliable state estimate.
- **Native C++ Performance**: The core mathematics are executed entirely in C++ via the Android NDK, preventing garbage collection stutters during high-speed robotic maneuvers.
- **Idiomatic Kotlin SDK**: No need to handle native pointers. Consume state updates effortlessly via Coroutines and StateFlow.
- **15-Dimensional State Tracking**: Tracks position, orientation, linear/angular velocity, and acceleration in real-time.
- **JitPack Ready**: Single-line dependency integration for any modern Android application.

---

## Architecture & Mathematical Foundation

This SDK uses an **Extended Kalman Filter (EKF)** to estimate the kinematic state of the device. Because GNSS data is highly accurate but updates slowly (e.g., 1Hz - 10Hz) and IMU data updates extremely fast (e.g., 100Hz+) but drifts over time, the EKF optimally blends them.

The library follows a high-performance **3-layer architecture**:
1.  **Kotlin SDK**: Modern, thread-safe API using Coroutines and `StateFlow`.
2.  **JNI Bridge**: Thin C++ layer for efficient data transfer.
3.  **C++ Core**: Heavy-lifting using **Eigen3** and optimized C++20.

### 1. The State Vector
The EKF tracks a **15-dimensional state vector** representing the robot/device in 3D space:

$$x = [p_x, p_y, p_z, \phi, \theta, \psi, v_x, v_y, v_z, \omega_x, \omega_y, \omega_z, a_x, a_y, a_z]^T$$

Where:
- $p_{x,y,z}$ = Position (ENU coordinates)
- $\phi, \theta, \psi$ = Orientation (Roll, Pitch, Yaw)
- $v_{x,y,z}$ = Linear Velocity
- $\omega_{x,y,z}$ = Angular Velocity
- $a_{x,y,z}$ = Linear Acceleration

### 2. The Prediction Step (Kinematic Model)
When new IMU data arrives, the filter predicts the new state based on the time elapsed ($\Delta t$). It integrates velocity into position, and acceleration into velocity:

$$x_{k|k-1} = f(x_{k-1|k-1})$$

It then propagates the error covariance ($P$) using the state transition Jacobian ($F$) and process noise ($Q$):

$$P_{k|k-1} = F P_{k-1|k-1} F^T + Q$$

### 3. The Update Step (Sensor Correction)
When an absolute measurement arrives (like an RTK GNSS coordinate), the filter corrects its prediction.

It calculates the **Innovation ($y$)**, which is the difference between the actual sensor reading ($z$) and the predicted sensor reading ($Hx$):

$$y = z - H x_{k|k-1}$$

It calculates the **Kalman Gain ($K$)**, which determines how much to "trust" the new sensor reading versus the internal prediction, based on the sensor's covariance/accuracy ($R$):

$$S = H P_{k|k-1} H^T + R$$
$$K = P_{k|k-1} H^T S^{-1}$$

Finally, it updates the state and the covariance matrix:

$$x_{k|k} = x_{k|k-1} + K y$$
$$P_{k|k} = (I - K H) P_{k|k-1}$$

---

## Installation

### 1. Add JitPack repository
In your `settings.gradle.kts`:

```kotlin
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
        maven { url = uri("https://jitpack.io") }
    }
}
```

### 2. Add Dependency
In your module `build.gradle.kts`:

```kotlin
dependencies {
    implementation("com.github.MohammadNouri5700:android-rtk-robot-kalman:1.0.0")
}
```

---

## Usage

### Basic Setup
```kotlin
// Initialize the engine
val rtkEngine = RtkRobotEngine()

// Observe the fused state
lifecycleScope.launch {
    rtkEngine.engineState.collect { state ->
        state?.let { 
            // Access fused Position (x,y,z), Orientation (roll,pitch,yaw), and Velocity
            Log.d("RTK", "Current Position: ${it.x}, ${it.y}")
        }
    }
}
```

### Feeding Sensors
```kotlin
// Push IMU data (typically from 50Hz to 400Hz)
rtkEngine.pushIMU(accelerometerArray, gyroscopeArray)

// Push GNSS data (typically from 1Hz to 10Hz)
rtkEngine.pushGNSS(latMeters, lonMeters, altMeters, accuracy)
```

### Cleaning Up
```kotlin
override fun onDestroy() {
    super.onDestroy()
    rtkEngine.close() // Important: Release native resources
}
```

---

## License
This project is licensed under the Apache License, Version 2.0. See the LICENSE file for details.
