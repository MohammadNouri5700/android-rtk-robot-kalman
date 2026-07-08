# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.5] - 2026-07-08

### Added
- **Native EKF Core**: High-performance 15-state Extended Kalman Filter implemented in C++20.
- **RTK GNSS Support**: Integrated handling for RTKLIB-based high-precision positioning.
- **Outlier Rejection**: Mahalanobis distance-based rejection for noisy GNSS measurements.

### Changed
- **.gitignore Refactor**: Comprehensive reorganization of ignored files to improve project cleanliness and support multiple IDEs (Android Studio, VS Code, CLion).
- **README Overhaul**: Added detailed technical documentation, mathematical models, and updated integration guides.

### Fixed
- Improved numerical stability for state covariance updates during high-frequency IMU processing.
- Optimized JNI data transfer overhead between Kotlin and C++ layers.
