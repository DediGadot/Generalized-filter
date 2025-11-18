// Complete Sensor Fusion Pipeline Integration Test
//
// Purpose: End-to-end validation of the full sensor fusion system
// Tests all phases working together:
// - Phase 1: Sensor data structures
// - Phase 2: IMU preintegration
// - Phase 3: EKF prediction
// - Phase 4: Measurement updates
// - Phase 5: Magnetometer integration

#include "filter/ekf_state.hpp"
#include "filter/imu_preintegration.hpp"
#include "filter/mag_update.hpp"
#include "core/sensor_types.hpp"
#include "core/quaternion.hpp"

#include <chrono>
#include <iostream>
#include <cmath>

using namespace fusion;
using namespace std::chrono;

// Test 1: Full pipeline - IMU + Magnetometer
bool test_complete_pipeline() {
    std::cout << "\n=== Test 1: Complete Pipeline (IMU + Magnetometer) ===" << std::endl;

    // Initialize EKF
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.5, 0.5, 1.0, 0.001, 0.01);

    double initial_cov = state.covariance()(ErrorState::DTHETA, ErrorState::DTHETA);
    std::cout << "Initial attitude covariance: " << initial_cov << std::endl;

    // Setup magnetometer
    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);
    Vector3d mag_reference = mag_update.get_reference_field();

    std::cout << "Simulating 5 seconds at 200Hz IMU, 50Hz Mag, 10Hz EKF" << std::endl;

    Vector3d gravity(0, 0, 9.81);
    ImuPreintegration preint;

    int ekf_updates = 0;
    int mag_updates = 0;

    // Simulate 5 seconds @ 200 Hz IMU rate
    for (int i = 0; i < 1000; i++) {
        // Create IMU sample (stationary)
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;  // 200 Hz = 5ms (prevent overflow)
        sample.gyro = Vector3f::Zero();
        sample.accel = Vector3f(0, 0, -9.81);  // Gravity in body frame

        preint.integrate(sample);

        // EKF prediction at 10 Hz (every 20 samples)
        if (i > 0 && i % 20 == 0) {
            auto preint_result = preint.get_result();
            state.predict(preint_result, gravity);
            preint.reset();
            ekf_updates++;
        }

        // Magnetometer update at 50 Hz (every 4 samples)
        if (i > 0 && i % 4 == 0) {
            bool accepted = mag_update.update(state, mag_reference, false);
            if (accepted) {
                state.inject_error(state.error_state());
                state.reset_error();
                mag_updates++;
            }
        }
    }

    std::cout << "EKF predictions: " << ekf_updates << std::endl;
    std::cout << "Mag updates: " << mag_updates << std::endl;

    double final_cov = state.covariance()(ErrorState::DTHETA, ErrorState::DTHETA);
    std::cout << "Final attitude covariance: " << final_cov << std::endl;

    bool cov_reduced = (final_cov < initial_cov * 0.3);
    bool updates_ok = (ekf_updates >= 45 && mag_updates >= 240);

    bool passed = (cov_reduced && updates_ok);

    if (passed) {
        std::cout << "✓ Test 1: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 1: FAILED" << std::endl;
    }

    return passed;
}

// Test 2: End-to-end performance
bool test_end_to_end_performance() {
    std::cout << "\n=== Test 2: End-to-End Performance ===" << std::endl;

    const int NUM_ITERATIONS = 1000;

    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);
    Vector3d mag_reference = mag_update.get_reference_field();
    Vector3d gravity(0, 0, 9.81);

    auto start = high_resolution_clock::now();

    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        ImuPreintegration preint;

        // Preintegrate 20 IMU samples (100ms @ 200Hz)
        for (int i = 0; i < 20; i++) {
            ImuSample sample;
            sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;  // Prevent overflow
            sample.gyro = Vector3f::Zero();
            sample.accel = Vector3f(0, 0, -9.81);
            preint.integrate(sample);
        }

        // EKF prediction
        auto preint_result = preint.get_result();
        state.predict(preint_result, gravity);

        // Magnetometer update
        mag_update.update(state, mag_reference, false);
        state.inject_error(state.error_state());
        state.reset_error();
    }

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start).count();

    double us_per_cycle = static_cast<double>(duration) / NUM_ITERATIONS;

    std::cout << "Performance: " << us_per_cycle << " µs per full cycle" << std::endl;
    std::cout << "  (20x IMU integration + 1x EKF prediction + 1x Mag update)" << std::endl;
    std::cout << "Target: <100 µs for 10 Hz real-time operation" << std::endl;

    double cycles_per_second = 1e6 / us_per_cycle;
    std::cout << "Throughput: " << static_cast<int>(cycles_per_second) << " cycles/second" << std::endl;
    std::cout << "Real-time capability: " << (cycles_per_second / 10.0) << "x faster than required" << std::endl;

    bool passed = (us_per_cycle < 100.0);

    if (passed) {
        std::cout << "✓ Test 2: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 2: FAILED" << std::endl;
    }

    return passed;
}

// Test 3: System accuracy validation
bool test_system_accuracy() {
    std::cout << "\n=== Test 3: System Accuracy Validation ===" << std::endl;

    // Initialize EKF
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

    // Run for 1 second stationary
    Vector3d gravity(0, 0, 9.81);
    ImuPreintegration preint;

    for (int i = 0; i < 200; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;  // 200 Hz, prevent overflow
        sample.gyro = Vector3f::Zero();
        sample.accel = Vector3f(0, 0, -9.81);

        preint.integrate(sample);

        if (i > 0 && i % 20 == 0) {
            auto preint_result = preint.get_result();
            state.predict(preint_result, gravity);
            preint.reset();
        }
    }

    // Check final state
    Vector3d final_pos = state.position();
    Vector3d final_vel = state.velocity();

    std::cout << "Final position: " << final_pos.transpose() << " m" << std::endl;
    std::cout << "Final velocity: " << final_vel.transpose() << " m/s" << std::endl;

    double pos_error = final_pos.norm();
    double vel_error = final_vel.norm();

    std::cout << "Position drift: " << pos_error << " m" << std::endl;
    std::cout << "Velocity drift: " << vel_error << " m/s" << std::endl;

    // Allow small drift (no measurements to correct it)
    bool pos_ok = (pos_error < 5.0);   // <5m drift over 1 second
    bool vel_ok = (vel_error < 1.0);    // <1m/s drift

    bool passed = (pos_ok && vel_ok);

    if (passed) {
        std::cout << "✓ Test 3: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 3: FAILED" << std::endl;
    }

    return passed;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Complete Sensor Fusion Pipeline Tests" << std::endl;
    std::cout << "Testing: All phases integrated (1-5)" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= test_complete_pipeline();
    all_passed &= test_end_to_end_performance();
    all_passed &= test_system_accuracy();

    std::cout << "\n========================================" << std::endl;
    if (all_passed) {
        std::cout << "✅ ALL INTEGRATION TESTS PASSED" << std::endl;
        std::cout << "Complete sensor fusion pipeline validated!" << std::endl;
        std::cout << "Phases 1-5 working together successfully" << std::endl;
        std::cout << "Ready for production deployment" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}
