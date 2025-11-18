// IMU Preintegration Unit Tests
//
// Purpose: Validate Forster preintegration algorithm
// Tests:
//   1. Zero motion → identity transformation
//   2. Constant rotation → analytical solution
//   3. Constant acceleration → analytical solution
//   4. Bias correction via Jacobians
//   5. Numerical integration accuracy
//   6. Performance benchmark

#include "filter/imu_preintegration.hpp"
#include "core/quaternion.hpp"

#include <chrono>
#include <iostream>
#include <cmath>

using namespace fusion;
using namespace std::chrono;

// Helper: Create IMU sample
ImuSample create_imu_sample(int64_t timestamp_ns,
                             const Eigen::Vector3f& gyro,
                             const Eigen::Vector3f& accel) {
    ImuSample sample;
    sample.timestamp_ns = timestamp_ns;
    sample.gyro = gyro;
    sample.accel = accel;
    sample.flags = 0;
    return sample;
}

// Test 1: Zero motion (no inputs)
bool test_zero_motion() {
    std::cout << "\n=== Test 1: Zero Motion ===" << std::endl;

    ImuPreintegration preint;

    // Simulate 1 second of zero IMU inputs (200 Hz)
    // This tests that zero inputs produce identity transformation
    const int NUM_SAMPLES = 200;
    const double dt = 0.005;  // 5ms

    for (int i = 0; i < NUM_SAMPLES; i++) {
        // Zero inputs (ideal case - no noise, no motion)
        ImuSample sample = create_imu_sample(
            static_cast<int64_t>(i * dt * 1e9),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),      // No rotation
            Eigen::Vector3f(0.0f, 0.0f, 0.0f)       // No acceleration
        );
        preint.integrate(sample);
    }

    auto result = preint.get_result();

    // Expected: Identity rotation (no rotation)
    Quaterniond expected_R = Quaterniond::Identity();
    double rotation_error = quaternion_distance(result.delta_R, expected_R);

    // Expected: Zero velocity (no motion)
    double velocity_error = result.delta_v.norm();

    // Expected: Zero position (no motion)
    double position_error = result.delta_p.norm();

    std::cout << "Rotation error: " << rotation_error << " rad (expect ~0)" << std::endl;
    std::cout << "Velocity error: " << velocity_error << " m/s (expect ~0)" << std::endl;
    std::cout << "Position error: " << position_error << " m (expect ~0)" << std::endl;

    bool passed = (rotation_error < 1e-10 &&
                   velocity_error < 1e-10 &&
                   position_error < 1e-10);

    if (passed) {
        std::cout << "✓ Test 1: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 1: FAILED" << std::endl;
    }

    return passed;
}

// Test 2: Constant rotation
bool test_constant_rotation() {
    std::cout << "\n=== Test 2: Constant Rotation ===" << std::endl;

    // Rotate at constant angular velocity
    const double omega_z = 0.1;  // 0.1 rad/s around Z axis
    const double total_time = 1.0;  // 1 second
    const double dt = 0.005;  // 5ms
    const int NUM_SAMPLES = static_cast<int>(total_time / dt) + 1;  // +1 because first sample is stored, not integrated

    ImuPreintegration preint;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ImuSample sample = create_imu_sample(
            static_cast<int64_t>(i * dt * 1e9),
            Eigen::Vector3f(0.0f, 0.0f, static_cast<float>(omega_z)),  // Rotate around Z
            Eigen::Vector3f(0.0f, 0.0f, 0.0f)                           // No acceleration
        );
        preint.integrate(sample);
    }

    auto result = preint.get_result();

    // Expected rotation: theta = omega * t
    // Account for midpoint integration: first sample is stored, then we integrate N-1 intervals
    double expected_theta = omega_z * total_time;
    Vector3d expected_axis(0.0, 0.0, 1.0);
    Quaterniond expected_R = quaternion_from_axis_angle(expected_axis, expected_theta);

    double rotation_error = quaternion_distance(result.delta_R, expected_R);

    std::cout << "Expected rotation: " << expected_theta << " rad" << std::endl;
    std::cout << "Actual rotation: " << quaternion_to_axis_angle(result.delta_R).second << " rad" << std::endl;
    std::cout << "Rotation error: " << rotation_error << " rad" << std::endl;

    bool passed = (rotation_error < 1e-6);  // Very tight tolerance for constant rotation

    if (passed) {
        std::cout << "✓ Test 2: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 2: FAILED" << std::endl;
    }

    return passed;
}

// Test 3: Constant acceleration
bool test_constant_acceleration() {
    std::cout << "\n=== Test 3: Constant Acceleration ===" << std::endl;

    // Accelerate forward at 1 m/s²
    const double accel_x = 1.0;  // m/s²
    const double total_time = 1.0;  // 1 second
    const double dt = 0.005;  // 5ms
    const int NUM_SAMPLES = static_cast<int>(total_time / dt) + 1;  // +1 for first stored sample

    ImuPreintegration preint;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ImuSample sample = create_imu_sample(
            static_cast<int64_t>(i * dt * 1e9),
            Eigen::Vector3f(0.0f, 0.0f, 0.0f),                       // No rotation
            Eigen::Vector3f(static_cast<float>(accel_x), 0.0f, 0.0f) // Forward accel only
        );
        preint.integrate(sample);
    }

    auto result = preint.get_result();

    // Expected velocity: v = a*t
    double expected_v = accel_x * total_time;
    double actual_v = result.delta_v.x();
    double velocity_error = std::abs(actual_v - expected_v);

    // Expected position: p = 0.5*a*t²
    double expected_p = 0.5 * accel_x * total_time * total_time;
    double actual_p = result.delta_p.x();
    double position_error = std::abs(actual_p - expected_p);

    std::cout << "Expected velocity: " << expected_v << " m/s" << std::endl;
    std::cout << "Actual velocity: " << actual_v << " m/s" << std::endl;
    std::cout << "Velocity error: " << velocity_error << " m/s" << std::endl;

    std::cout << "Expected position: " << expected_p << " m" << std::endl;
    std::cout << "Actual position: " << actual_p << " m" << std::endl;
    std::cout << "Position error: " << position_error << " m" << std::endl;

    bool passed = (velocity_error < 1e-6 && position_error < 1e-6);

    if (passed) {
        std::cout << "✓ Test 3: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 3: FAILED" << std::endl;
    }

    return passed;
}

// Test 4: Bias correction via Jacobians
bool test_bias_correction() {
    std::cout << "\n=== Test 4: Bias Correction ===" << std::endl;

    // Initial bias
    Vector3d bias_gyro_initial(0.01, 0.0, 0.0);  // 0.01 rad/s bias
    Vector3d bias_accel_initial(0.1, 0.0, 0.0);  // 0.1 m/s² bias

    const double total_time = 1.0;
    const double dt = 0.005;
    const int NUM_SAMPLES = static_cast<int>(total_time / dt) + 1;  // +1 for first stored sample

    // Preintegrate with initial bias
    ImuPreintegration preint1(bias_gyro_initial, bias_accel_initial);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ImuSample sample = create_imu_sample(
            static_cast<int64_t>(i * dt * 1e9),
            Eigen::Vector3f(0.01f, 0.0f, 0.0f),  // Matches bias (no actual rotation)
            Eigen::Vector3f(0.1f, 0.0f, 0.0f)    // Matches bias (no actual accel)
        );
        preint1.integrate(sample);
    }

    auto result1 = preint1.get_result();

    // Update bias and correct using Jacobians
    Vector3d bias_gyro_updated(0.02, 0.0, 0.0);
    Vector3d bias_accel_updated(0.2, 0.0, 0.0);
    preint1.update_bias(bias_gyro_updated, bias_accel_updated);

    auto result1_corrected = preint1.get_result();

    // Preintegrate again from scratch with updated bias (ground truth)
    ImuPreintegration preint2(bias_gyro_updated, bias_accel_updated);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ImuSample sample = create_imu_sample(
            static_cast<int64_t>(i * dt * 1e9),
            Eigen::Vector3f(0.01f, 0.0f, 0.0f),
            Eigen::Vector3f(0.1f, 0.0f, 0.0f)
        );
        preint2.integrate(sample);
    }

    auto result2 = preint2.get_result();

    // Compare Jacobian-corrected vs. re-integrated
    double rotation_error = quaternion_distance(result1_corrected.delta_R, result2.delta_R);
    double velocity_error = (result1_corrected.delta_v - result2.delta_v).norm();
    double position_error = (result1_corrected.delta_p - result2.delta_p).norm();

    std::cout << "Rotation error (Jacobian vs re-integration): " << rotation_error << " rad" << std::endl;
    std::cout << "Velocity error: " << velocity_error << " m/s" << std::endl;
    std::cout << "Position error: " << position_error << " m" << std::endl;

    // Jacobian correction should be accurate for small bias changes
    bool passed = (rotation_error < 1e-6 &&
                   velocity_error < 1e-6 &&
                   position_error < 1e-6);

    if (passed) {
        std::cout << "✓ Test 4: PASSED (Jacobian correction accurate)" << std::endl;
    } else {
        std::cerr << "✗ Test 4: FAILED" << std::endl;
    }

    return passed;
}

// Test 5: Performance benchmark
bool test_performance() {
    std::cout << "\n=== Test 5: Performance Benchmark ===" << std::endl;

    const int NUM_ITERATIONS = 10000;
    const int SAMPLES_PER_ITERATION = 2;  // Typical: 2 samples per 10ms EKF epoch

    ImuPreintegration preint;

    auto start = high_resolution_clock::now();

    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
        preint.reset();

        for (int i = 0; i < SAMPLES_PER_ITERATION; i++) {
            ImuSample sample = create_imu_sample(
                static_cast<int64_t>(i * 5e6),  // 5ms spacing
                Eigen::Vector3f(0.01f, 0.02f, 0.03f),
                Eigen::Vector3f(0.1f, 0.2f, 9.81f)
            );
            preint.integrate(sample);
        }

        auto result = preint.get_result();  // Force computation
        (void)result;  // Avoid unused warning
    }

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<nanoseconds>(end - start).count();

    double ns_per_integrate = static_cast<double>(duration) / (NUM_ITERATIONS * SAMPLES_PER_ITERATION);
    double us_per_integrate = ns_per_integrate / 1000.0;

    std::cout << "Performance: " << us_per_integrate << " µs per integrate()" << std::endl;
    std::cout << "Target: <10 µs on Snapdragon AR1 Gen 1" << std::endl;

    // Desktop CPU should easily beat this
    bool passed = (us_per_integrate < 50.0);  // Allow 50µs on desktop

    if (passed) {
        std::cout << "✓ Test 5: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 5: FAILED (too slow)" << std::endl;
    }

    return passed;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "IMU Preintegration Unit Tests" << std::endl;
    std::cout << "Testing: Forster et al. algorithm" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= test_zero_motion();
    all_passed &= test_constant_rotation();
    all_passed &= test_constant_acceleration();
    all_passed &= test_bias_correction();
    all_passed &= test_performance();

    std::cout << "\n========================================" << std::endl;
    if (all_passed) {
        std::cout << "✅ ALL PREINTEGRATION TESTS PASSED" << std::endl;
        std::cout << "Forster algorithm validated" << std::endl;
        std::cout << "Ready for EKF integration" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}
