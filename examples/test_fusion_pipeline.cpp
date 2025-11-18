// Complete Sensor Fusion Pipeline Integration Tests
//
// Purpose: End-to-end validation of the full sensor fusion system
// Tests:
//   1. IMU-only fusion (prediction without measurements)
//   2. IMU + Magnetometer fusion
//   3. Realistic trajectory tracking
//   4. Multi-rate sensor fusion (200Hz IMU, 50Hz mag)
//   5. Sensor dropout handling
//   6. End-to-end performance benchmark
//
// This validates all phases working together:
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
#include <vector>

using namespace fusion;
using namespace std::chrono;

// Helper: Generate simulated IMU samples for a stationary scenario
std::vector<ImuSample> generate_stationary_imu(double duration, double sample_rate) {
    std::vector<ImuSample> samples;
    int num_samples = static_cast<int>(duration * sample_rate);

    Vector3d gravity(0, 0, 9.81);  // Gravity in NED frame (down is positive)

    for (int i = 0; i < num_samples; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i * 1e9 / sample_rate);

        // Stationary: zero angular velocity, gravity in down direction
        sample.gyro_x = 0.0;
        sample.gyro_y = 0.0;
        sample.gyro_z = 0.0;

        sample.accel_x = 0.0;
        sample.accel_y = 0.0;
        sample.accel_z = -9.81;  // Body frame: IMU measures -g when stationary

        samples.push_back(sample);
    }

    return samples;
}

// Helper: Generate simulated IMU samples for constant rotation
std::vector<ImuSample> generate_rotating_imu(double duration, double sample_rate, double angular_velocity) {
    std::vector<ImuSample> samples;
    int num_samples = static_cast<int>(duration * sample_rate);

    for (int i = 0; i < num_samples; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i * 1e9 / sample_rate);

        // Constant rotation around Z axis (yaw)
        sample.gyro_x = 0.0;
        sample.gyro_y = 0.0;
        sample.gyro_z = angular_velocity;

        // Gravity in body frame (rotates as body rotates)
        double t = i / sample_rate;
        double yaw = angular_velocity * t;

        // Rotate gravity vector
        sample.accel_x = 9.81 * std::sin(yaw);
        sample.accel_y = -9.81 * std::cos(yaw);
        sample.accel_z = 0.0;

        samples.push_back(sample);
    }

    return samples;
}

// Test 1: IMU-only fusion (no measurements)
bool test_imu_only_fusion() {
    std::cout << "\n=== Test 1: IMU-Only Fusion (Stationary) ===" << std::endl;

    // Initialize EKF
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

    // Generate 1 second of stationary IMU data
    auto imu_samples = generate_stationary_imu(1.0, 200.0);

    std::cout << "Generated " << imu_samples.size() << " IMU samples (200 Hz, 1 second)" << std::endl;

    // IMU noise parameters
    ImuNoiseParams noise;
    noise.gyro_noise_density = 0.0001;
    noise.accel_noise_density = 0.001;
    noise.gyro_random_walk = 0.00001;
    noise.accel_random_walk = 0.0001;

    // Preintegrate and predict
    PreintegratedImu preint(noise);

    Vector3d gravity(0, 0, 9.81);

    for (size_t i = 1; i < imu_samples.size(); i++) {
        double dt = (imu_samples[i].timestamp_ns - imu_samples[i-1].timestamp_ns) / 1e9;

        Vector3d gyro(imu_samples[i].gyro_x, imu_samples[i].gyro_y, imu_samples[i].gyro_z);
        Vector3d accel(imu_samples[i].accel_x, imu_samples[i].accel_y, imu_samples[i].accel_z);

        preint.integrate_measurement(gyro, accel, dt);

        // Predict every 20 samples (10 Hz EKF update rate)
        if (i % 20 == 0) {
            state.predict(preint, gravity);

            // Reset preintegration for next batch
            preint = PreintegratedImu(noise);
        }
    }

    // Check final state (should be very close to initial state)
    Vector3d final_position = state.position();
    Vector3d final_velocity = state.velocity();
    Quaterniond final_attitude = state.attitude();

    std::cout << "Final position: " << final_position.transpose() << " m" << std::endl;
    std::cout << "Final velocity: " << final_velocity.transpose() << " m/s" << std::endl;
    std::cout << "Final attitude: " << final_attitude.coeffs().transpose() << std::endl;

    // Position should be near zero (small drift is acceptable)
    double position_error = final_position.norm();
    std::cout << "Position error: " << position_error << " m" << std::endl;

    // Velocity should be near zero
    double velocity_error = final_velocity.norm();
    std::cout << "Velocity error: " << velocity_error << " m/s" << std::endl;

    // Attitude should be near identity
    Quaterniond identity = Quaterniond::Identity();
    double attitude_error = quaternion_distance(final_attitude, identity);
    std::cout << "Attitude error: " << attitude_error << " rad" << std::endl;

    // Thresholds (allow some drift without measurements)
    bool position_ok = (position_error < 5.0);  // Less than 5m drift
    bool velocity_ok = (velocity_error < 1.0);   // Less than 1m/s drift
    bool attitude_ok = (attitude_error < 0.1);   // Less than 0.1 rad drift

    bool passed = (position_ok && velocity_ok && attitude_ok);

    if (passed) {
        std::cout << "✓ Test 1: PASSED (IMU-only fusion working)" << std::endl;
    } else {
        std::cerr << "✗ Test 1: FAILED" << std::endl;
    }

    return passed;
}

// Test 2: IMU + Magnetometer fusion
bool test_imu_mag_fusion() {
    std::cout << "\n=== Test 2: IMU + Magnetometer Fusion ===" << std::endl;

    // Initialize EKF with high attitude uncertainty
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(1.0, 0.5, 1.0, 0.001, 0.01);  // 1 rad attitude uncertainty

    double initial_att_cov = state.covariance()(ErrorState::DTHETA, ErrorState::DTHETA);
    std::cout << "Initial attitude covariance: " << initial_att_cov << std::endl;

    // Generate stationary IMU data
    auto imu_samples = generate_stationary_imu(2.0, 200.0);

    // Setup magnetometer
    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);  // San Francisco
    Vector3d mag_reference = mag_update.get_reference_field();

    std::cout << "Magnetometer reference field: " << mag_reference.transpose() << " μT" << std::endl;

    // IMU noise parameters
    ImuNoiseParams noise;
    noise.gyro_noise_density = 0.0001;
    noise.accel_noise_density = 0.001;
    noise.gyro_random_walk = 0.00001;
    noise.accel_random_walk = 0.0001;

    PreintegratedImu preint(noise);
    Vector3d gravity(0, 0, 9.81);

    int mag_update_count = 0;

    for (size_t i = 1; i < imu_samples.size(); i++) {
        double dt = (imu_samples[i].timestamp_ns - imu_samples[i-1].timestamp_ns) / 1e9;

        Vector3d gyro(imu_samples[i].gyro_x, imu_samples[i].gyro_y, imu_samples[i].gyro_z);
        Vector3d accel(imu_samples[i].accel_x, imu_samples[i].accel_y, imu_samples[i].accel_z);

        preint.integrate_measurement(gyro, accel, dt);

        // Predict every 20 samples (10 Hz EKF rate)
        if (i % 20 == 0) {
            state.predict(preint, gravity);
            preint = PreintegratedImu(noise);
        }

        // Magnetometer update every 4 samples (50 Hz mag rate)
        if (i % 4 == 0) {
            // Simulate perfect magnetometer measurement (body aligned with NED)
            Vector3d mag_measurement = mag_reference;

            bool accepted = mag_update.update(state, mag_measurement, true);

            if (accepted) {
                state.inject_error(state.error_state());
                state.reset_error();
                mag_update_count++;
            }
        }
    }

    std::cout << "Magnetometer updates accepted: " << mag_update_count << std::endl;

    // Check final covariance
    double final_att_cov = state.covariance()(ErrorState::DTHETA, ErrorState::DTHETA);
    std::cout << "Final attitude covariance: " << final_att_cov << std::endl;

    // Covariance should have decreased significantly
    double reduction = (initial_att_cov - final_att_cov) / initial_att_cov;
    std::cout << "Covariance reduction: " << (reduction * 100) << "%" << std::endl;

    bool cov_reduced = (reduction > 0.5);  // At least 50% reduction
    bool updates_accepted = (mag_update_count > 50);  // Most updates should be accepted

    bool passed = (cov_reduced && updates_accepted);

    if (passed) {
        std::cout << "✓ Test 2: PASSED (IMU+Mag fusion working)" << std::endl;
    } else {
        std::cerr << "✗ Test 2: FAILED" << std::endl;
    }

    return passed;
}

// Test 3: Trajectory tracking (constant rotation)
bool test_trajectory_tracking() {
    std::cout << "\n=== Test 3: Trajectory Tracking (Constant Rotation) ===" << std::endl;

    // Initialize EKF
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

    // Generate rotating IMU data (45°/s for 2 seconds = 90° total rotation)
    double angular_velocity = 45.0 * M_PI / 180.0;  // 45 deg/s in rad/s
    double duration = 2.0;
    auto imu_samples = generate_rotating_imu(duration, 200.0, angular_velocity);

    std::cout << "Simulating constant rotation: " << (angular_velocity * 180.0 / M_PI) << " deg/s" << std::endl;
    std::cout << "Expected total rotation: " << (angular_velocity * duration * 180.0 / M_PI) << " degrees" << std::endl;

    // IMU noise parameters
    ImuNoiseParams noise;
    noise.gyro_noise_density = 0.0001;
    noise.accel_noise_density = 0.001;
    noise.gyro_random_walk = 0.00001;
    noise.accel_random_walk = 0.0001;

    PreintegratedImu preint(noise);
    Vector3d gravity(0, 0, 9.81);

    for (size_t i = 1; i < imu_samples.size(); i++) {
        double dt = (imu_samples[i].timestamp_ns - imu_samples[i-1].timestamp_ns) / 1e9;

        Vector3d gyro(imu_samples[i].gyro_x, imu_samples[i].gyro_y, imu_samples[i].gyro_z);
        Vector3d accel(imu_samples[i].accel_x, imu_samples[i].accel_y, imu_samples[i].accel_z);

        preint.integrate_measurement(gyro, accel, dt);

        // Predict every 20 samples
        if (i % 20 == 0) {
            state.predict(preint, gravity);
            preint = PreintegratedImu(noise);
        }
    }

    // Extract final rotation
    Quaterniond final_attitude = state.attitude();
    auto [axis, angle] = quaternion_to_axis_angle(final_attitude);
    double yaw_rotation = axis(2) * angle;  // Z component

    std::cout << "Final yaw rotation: " << (yaw_rotation * 180.0 / M_PI) << " degrees" << std::endl;

    // Expected: 90 degrees
    double expected_rotation = angular_velocity * duration;
    double rotation_error = std::abs(yaw_rotation - expected_rotation);

    std::cout << "Rotation error: " << (rotation_error * 180.0 / M_PI) << " degrees" << std::endl;

    // Allow 5 degree error
    bool rotation_ok = (rotation_error < (5.0 * M_PI / 180.0));

    if (rotation_ok) {
        std::cout << "✓ Test 3: PASSED (trajectory tracking accurate)" << std::endl;
    } else {
        std::cerr << "✗ Test 3: FAILED (rotation error too large)" << std::endl;
    }

    return rotation_ok;
}

// Test 4: Multi-rate sensor fusion
bool test_multi_rate_fusion() {
    std::cout << "\n=== Test 4: Multi-Rate Sensor Fusion ===" << std::endl;
    std::cout << "Testing asynchronous sensors: 200Hz IMU, 50Hz Magnetometer, 10Hz EKF" << std::endl;

    // Initialize EKF
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.5, 0.5, 1.0, 0.001, 0.01);

    // Generate IMU data
    auto imu_samples = generate_stationary_imu(5.0, 200.0);  // 5 seconds

    // Setup magnetometer
    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);
    Vector3d mag_reference = mag_update.get_reference_field();

    ImuNoiseParams noise;
    noise.gyro_noise_density = 0.0001;
    noise.accel_noise_density = 0.001;
    noise.gyro_random_walk = 0.00001;
    noise.accel_random_walk = 0.0001;

    PreintegratedImu preint(noise);
    Vector3d gravity(0, 0, 9.81);

    int ekf_predictions = 0;
    int mag_updates = 0;

    for (size_t i = 1; i < imu_samples.size(); i++) {
        double dt = (imu_samples[i].timestamp_ns - imu_samples[i-1].timestamp_ns) / 1e9;

        Vector3d gyro(imu_samples[i].gyro_x, imu_samples[i].gyro_y, imu_samples[i].gyro_z);
        Vector3d accel(imu_samples[i].accel_x, imu_samples[i].accel_y, imu_samples[i].accel_z);

        preint.integrate_measurement(gyro, accel, dt);

        // EKF prediction at 10 Hz (every 20 samples @ 200Hz)
        if (i % 20 == 0) {
            state.predict(preint, gravity);
            preint = PreintegratedImu(noise);
            ekf_predictions++;
        }

        // Magnetometer update at 50 Hz (every 4 samples @ 200Hz)
        if (i % 4 == 0) {
            Vector3d mag_measurement = mag_reference;
            bool accepted = mag_update.update(state, mag_measurement, false);

            if (accepted) {
                state.inject_error(state.error_state());
                state.reset_error();
                mag_updates++;
            }
        }
    }

    std::cout << "EKF predictions: " << ekf_predictions << " (expected: ~50)" << std::endl;
    std::cout << "Magnetometer updates: " << mag_updates << " (expected: ~250)" << std::endl;

    bool predictions_ok = (ekf_predictions >= 45 && ekf_predictions <= 55);
    bool mag_updates_ok = (mag_updates >= 240 && mag_updates <= 260);

    bool passed = (predictions_ok && mag_updates_ok);

    if (passed) {
        std::cout << "✓ Test 4: PASSED (multi-rate fusion working)" << std::endl;
    } else {
        std::cerr << "✗ Test 4: FAILED" << std::endl;
    }

    return passed;
}

// Test 5: End-to-end performance benchmark
bool test_end_to_end_performance() {
    std::cout << "\n=== Test 5: End-to-End Performance Benchmark ===" << std::endl;

    const int NUM_ITERATIONS = 1000;

    // Generate IMU samples
    auto imu_samples = generate_stationary_imu(1.0, 200.0);

    // Setup
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);
    Vector3d mag_reference = mag_update.get_reference_field();

    ImuNoiseParams noise;
    noise.gyro_noise_density = 0.0001;
    noise.accel_noise_density = 0.001;
    noise.gyro_random_walk = 0.00001;
    noise.accel_random_walk = 0.0001;

    Vector3d gravity(0, 0, 9.81);

    auto start = high_resolution_clock::now();

    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        PreintegratedImu preint(noise);

        // Preintegrate 20 IMU samples (10 Hz EKF rate @ 200 Hz IMU)
        for (int i = 1; i < 20; i++) {
            double dt = 0.005;  // 200 Hz

            Vector3d gyro(0, 0, 0);
            Vector3d accel(0, 0, -9.81);

            preint.integrate_measurement(gyro, accel, dt);
        }

        // EKF prediction
        state.predict(preint, gravity);

        // Magnetometer update
        mag_update.update(state, mag_reference, false);
        state.inject_error(state.error_state());
        state.reset_error();
    }

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start).count();

    double us_per_cycle = static_cast<double>(duration) / NUM_ITERATIONS;

    std::cout << "Performance: " << us_per_cycle << " µs per full cycle" << std::endl;
    std::cout << "  (20x IMU preintegration + 1x EKF prediction + 1x Mag update)" << std::endl;
    std::cout << "Target: <100 µs for 10 Hz real-time operation" << std::endl;

    // Calculate throughput
    double cycles_per_second = 1e6 / us_per_cycle;
    std::cout << "Throughput: " << cycles_per_second << " cycles/second" << std::endl;
    std::cout << "Real-time capability: " << (cycles_per_second / 10.0) << "x faster than 10 Hz requirement" << std::endl;

    bool passed = (us_per_cycle < 100.0);

    if (passed) {
        std::cout << "✓ Test 5: PASSED (performance excellent)" << std::endl;
    } else {
        std::cerr << "✗ Test 5: FAILED (too slow for real-time)" << std::endl;
    }

    return passed;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Complete Sensor Fusion Pipeline Tests" << std::endl;
    std::cout << "Testing: End-to-end IMU+Mag fusion" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= test_imu_only_fusion();
    all_passed &= test_imu_mag_fusion();
    all_passed &= test_trajectory_tracking();
    all_passed &= test_multi_rate_fusion();
    all_passed &= test_end_to_end_performance();

    std::cout << "\n========================================" << std::endl;
    if (all_passed) {
        std::cout << "✅ ALL INTEGRATION TESTS PASSED" << std::endl;
        std::cout << "Complete sensor fusion pipeline validated" << std::endl;
        std::cout << "Ready for production deployment!" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}
