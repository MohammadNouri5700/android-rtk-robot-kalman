# Sets the minimum CMake version required for this project.
cmake_minimum_required(VERSION 3.22.1)

# Declares the project name.
project(rtklib_localization VERSION 1.0.3 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Hardware Acceleration and ABI Optimizations
if (ANDROID_ABI STREQUAL "arm64-v8a")
    add_compile_options(-O3 -march=armv8-a+simd -flto)
elseif (ANDROID_ABI STREQUAL "armeabi-v7a")
    add_compile_options(-O3 -march=armv7-a -mfloat-abi=softfp -mfpu=neon)
endif()

# Locate Eigen3 Math Library
get_filename_component(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party/eigen3" ABSOLUTE)

if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
    message(FATAL_ERROR "Eigen3 not found at ${EIGEN3_INCLUDE_DIR}. Please check the path.")
endif()

message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# GoogleTest Integration
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
)
# Forcing GTest to not use shared libraries on Windows if applicable,
# and disabling install targets to keep build clean.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# RTKLIB GNSS Configurations
add_definitions(-DENAGLO -DENAGAL -DENAQZS -DNFREQ=3)

# ==========================================================
# SDK ARCHITECTURE TARGETS
# ==========================================================

# 1. Static Library for GNSS Core
add_library(rtklib_core STATIC
        ../../../../core/src/rtklib_interface.cpp
        ../../../../third_party/RTKLIB/src/rtkcmn.c
)
target_include_directories(rtklib_core PUBLIC
        ../../../../core/include
        ../../../../third_party/RTKLIB/src
)

# 2. Shared Library for Android Native Bridge
add_library(rtk_robot_kalman_engine SHARED
        ../../../../core/src/sensor_fusion.cpp
        jni_bridge.cpp
)

# 3. Specify Header Directories for the Engine
target_include_directories(rtk_robot_kalman_engine PUBLIC
        ../../../../core/include
        ../../../../third_party/RTKLIB/src
)

# 4. Link Libraries
target_link_libraries(rtk_robot_kalman_engine
        rtklib_core
        android
        log
)

# 5. Unit Tests (C++)
add_executable(sensor_fusion_test
        ../../../../core/src/sensor_fusion.cpp
        ../../../../core/test/sensor_fusion_test.cpp
)
target_include_directories(sensor_fusion_test PUBLIC
        ../../../../core/include
        ../../../../third_party/RTKLIB/src
)
target_link_libraries(sensor_fusion_test
        rtklib_core
        GTest::gtest_main
        log
)
