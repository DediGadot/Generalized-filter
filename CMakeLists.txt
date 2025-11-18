# Multi-Sensor Fusion Filter - Root CMakeLists.txt
# Supports both desktop (x86_64) and Android NDK (ARM) builds
# Author: Based on DESIGN.md specifications
# Date: 2025-11-18

cmake_minimum_required(VERSION 3.22)
project(sensor_fusion CXX)

# C++20 standard (modern features, Eigen compatibility)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Platform detection
if(ANDROID)
    message(STATUS "Building for Android NDK (ARM)")
    set(PLATFORM_NAME "android")
    # Android-specific flags (Snapdragon AR1 Gen 1 optimization)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a -mtune=cortex-a76")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -Wall -Wextra")
else()
    message(STATUS "Building for Desktop (x86_64/ARM64)")
    set(PLATFORM_NAME "desktop")
    # Desktop optimization
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Eigen library (header-only, included in third_party)
set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/third_party/eigen)
if(NOT EXISTS ${EIGEN_INCLUDE_DIR})
    message(STATUS "Eigen not found, will be downloaded")
    # Note: In practice, download Eigen here or instruct user to do so
    # For now, assume it exists or will be added
endif()

include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Core library (shared between desktop and Android)
add_library(fusion_core STATIC
    src/core/quaternion.cpp
    src/core/sensor_types.cpp
    src/math/vector_math.cpp
    src/validation/synthetic_imu.cpp
    src/validation/synthetic_gnss.cpp
)

target_include_directories(fusion_core PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN_INCLUDE_DIR}
)

# Platform-specific libraries
if(ANDROID)
    # Find Android libraries
    find_library(log-lib log)
    find_library(android-lib android)

    target_link_libraries(fusion_core
        ${log-lib}
        ${android-lib}
    )
else()
    # Desktop: Enable testing with Google Test
    enable_testing()

    # Google Test
    set(GOOGLETEST_DIR ${CMAKE_SOURCE_DIR}/third_party/googletest)
    if(EXISTS ${GOOGLETEST_DIR})
        add_subdirectory(${GOOGLETEST_DIR})

        # Unit tests
        add_executable(test_quaternion tests/test_quaternion.cpp)
        target_link_libraries(test_quaternion fusion_core gtest gtest_main)
        add_test(NAME QuaternionTests COMMAND test_quaternion)

        add_executable(test_types tests/test_types.cpp)
        target_link_libraries(test_types fusion_core gtest gtest_main)
        add_test(NAME TypeTests COMMAND test_types)

        add_executable(test_synthetic tests/test_synthetic.cpp)
        target_link_libraries(test_synthetic fusion_core gtest gtest_main)
        add_test(NAME SyntheticTests COMMAND test_synthetic)
    endif()

    # Phase 0 validation executable
    add_executable(validate_phase0 examples/validate_phase0.cpp)
    target_link_libraries(validate_phase0 fusion_core)
endif()

# Print configuration summary
message(STATUS "========================================")
message(STATUS "Sensor Fusion Filter - Build Configuration")
message(STATUS "========================================")
message(STATUS "Platform: ${PLATFORM_NAME}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "Eigen Include: ${EIGEN_INCLUDE_DIR}")
message(STATUS "========================================")
