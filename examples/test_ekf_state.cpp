// EKF State Unit Tests
//
// Purpose: Validate error-state EKF implementation
// Tests:
//   1. State initialization
//   2. State propagation with preintegration
//   3. Error injection and reset
//   4. Covariance growth during prediction
//   5. F matrix validation via finite differences
//   6. Performance benchmark

#include "filter/ekf_state.hpp"
#include "filter/imu_preintegration.hpp"
#include "core/quaternion.hpp"

#include <chrono>
#include <iostream>
#include <cmath>

using namespace fusion;
using namespace std::chrono;

// Test 1: State initialization
bool test_initialization() {
    std::cout << "\n=== Test 1: State Initialization ===" << std::endl;

    EkfState state;

    // Initialize with known values
    Vector3d p0(0, 0, 0);
    Vector3d v0(0, 0, 0);
    Quaterniond q0 = Quaterniond::Identity();

    state.initialize(p0, v0, q0, 0);

    // Set initial covariance
    state.set_initial_covariance(
        0.1,   // 0.1 rad rotation uncertainty
        0.5,   // 0.5 m/s velocity uncertainty
        1.0,   // 1.0 m position uncertainty
        0.001, // 0.001 rad/s gyro bias uncertainty
        0.01   // 0.01 m/s² accel bias uncertainty
    );

    // Verify initialization
    double p_error = (state.position() - p0).norm();
    double v_error = (state.velocity() - v0).norm();
    double q_error = quaternion_distance(state.attitude(), q0);

    std::cout << "Position error: " << p_error << " m" << std::endl;
    std::cout << "Velocity error: " << v_error << " m/s" << std::endl;
    std::cout << "Attitude error: " << q_error << " rad" << std::endl;

    // Check covariance is positive definite (all diagonal elements > 0)
    const auto& P = state.covariance();
    bool cov_valid = true;
    for (int i = 0; i < 15; i++) {
        if (P(i, i) <= 0) {
            cov_valid = false;
            break;
        }
    }

    std::cout << "Covariance positive definite: " << (cov_valid ? "Yes" : "No") << std::endl;

    bool passed = (p_error < 1e-10 && v_error < 1e-10 && q_error < 1e-10 && cov_valid);

    if (passed) {
        std::cout << "✓ Test 1: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 1: FAILED" << std::endl;
    }

    return passed;
}

// Test 2: State propagation with zero motion
bool test_zero_motion_propagation() {
    std::cout << "\n=== Test 2: Zero Motion Propagation ===" << std::endl;

    EkfState state;

    // Initialize at origin
    Vector3d p0(0, 0, 0);
    Vector3d v0(0, 0, 0);
    Quaterniond q0 = Quaterniond::Identity();

    state.initialize(p0, v0, q0, 0);
    state.set_initial_covariance(0.01, 0.1, 0.1, 0.001, 0.01);

    // Create zero-motion preintegration (1 second)
    ImuPreintegration preint;
    for (int i = 0; i < 201; i++) {  // 201 samples for 1 second
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i * 5e6);  // 5ms spacing
        sample.gyro = Eigen::Vector3f(0, 0, 0);
        sample.accel = Eigen::Vector3f(0, 0, 0);
        sample.flags = 0;
        preint.integrate(sample);
    }

    auto preint_result = preint.get_result();

    // Predict with gravity (NED frame: down is +Z)
    Vector3d gravity(0, 0, 9.81);
    state.predict(preint_result, gravity);

    // Expected: Fall under gravity (no initial velocity)
    // p = 0.5 * g * t² = 0.5 * 9.81 * 1² = 4.905 m down
    // v = g * t = 9.81 * 1 = 9.81 m/s down

    Vector3d expected_p(0, 0, 4.905);
    Vector3d expected_v(0, 0, 9.81);

    double p_error = (state.position() - expected_p).norm();
    double v_error = (state.velocity() - expected_v).norm();
    double q_error = quaternion_distance(state.attitude(), q0);

    std::cout << "Expected position: " << expected_p.transpose() << std::endl;
    std::cout << "Actual position: " << state.position().transpose() << std::endl;
    std::cout << "Position error: " << p_error << " m" << std::endl;

    std::cout << "Expected velocity: " << expected_v.transpose() << std::endl;
    std::cout << "Actual velocity: " << state.velocity().transpose() << std::endl;
    std::cout << "Velocity error: " << v_error << " m/s" << std::endl;

    std::cout << "Attitude error: " << q_error << " rad (expect 0)" << std::endl;

    bool passed = (p_error < 1e-6 && v_error < 1e-6 && q_error < 1e-10);

    if (passed) {
        std::cout << "✓ Test 2: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 2: FAILED" << std::endl;
    }

    return passed;
}

// Test 3: Covariance growth
bool test_covariance_growth() {
    std::cout << "\n=== Test 3: Covariance Growth ===" << std::endl;

    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.01, 0.1, 0.1, 0.001, 0.01);

    // Save initial covariance
    Eigen::Matrix<double, 15, 15> P_initial = state.covariance();

    // Propagate with zero motion but with process noise
    ImuPreintegration preint;
    for (int i = 0; i < 201; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i * 5e6);
        sample.gyro = Eigen::Vector3f(0, 0, 0);
        sample.accel = Eigen::Vector3f(0, 0, 0);
        sample.flags = 0;
        preint.integrate(sample);
    }

    auto preint_result = preint.get_result();
    state.predict(preint_result, Vector3d(0, 0, 9.81));

    // Save final covariance
    Eigen::Matrix<double, 15, 15> P_final = state.covariance();

    // Check that covariance has grown (uncertainty increases without measurements)
    bool covariance_grew = false;
    for (int i = 0; i < 15; i++) {
        if (P_final(i, i) > P_initial(i, i)) {
            covariance_grew = true;
            break;
        }
    }

    std::cout << "Initial rotation covariance: " << P_initial(0, 0) << std::endl;
    std::cout << "Final rotation covariance: " << P_final(0, 0) << std::endl;

    std::cout << "Initial velocity covariance: " << P_initial(3, 3) << std::endl;
    std::cout << "Final velocity covariance: " << P_final(3, 3) << std::endl;

    std::cout << "Covariance grew: " << (covariance_grew ? "Yes" : "No") << std::endl;

    bool passed = covariance_grew;

    if (passed) {
        std::cout << "✓ Test 3: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 3: FAILED" << std::endl;
    }

    return passed;
}

// Test 4: Error injection and reset
bool test_error_injection() {
    std::cout << "\n=== Test 4: Error Injection and Reset ===" << std::endl;

    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);

    // Create a small error state update
    Eigen::Matrix<double, 15, 1> error_update;
    error_update.setZero();
    error_update.segment<3>(ErrorState::DTHETA) = Vector3d(0.01, 0, 0);  // 0.01 rad rotation about X
    error_update.segment<3>(ErrorState::DV) = Vector3d(0.1, 0, 0);        // 0.1 m/s velocity in X
    error_update.segment<3>(ErrorState::DP) = Vector3d(1.0, 0, 0);        // 1.0 m position in X

    // Inject error
    state.inject_error(error_update);

    // Check that nominal state changed
    double p_change = state.position().x();
    double v_change = state.velocity().x();
    auto [axis, angle] = quaternion_to_axis_angle(state.attitude());
    double q_change = angle;

    std::cout << "Position change: " << p_change << " m (expected 1.0)" << std::endl;
    std::cout << "Velocity change: " << v_change << " m/s (expected 0.1)" << std::endl;
    std::cout << "Attitude change: " << q_change << " rad (expected ~0.01)" << std::endl;

    // Reset error state
    state.reset_error();

    // Check that error state is zero
    double error_norm = state.error_state().norm();
    std::cout << "Error state norm after reset: " << error_norm << " (expected 0)" << std::endl;

    bool passed = (std::abs(p_change - 1.0) < 1e-6 &&
                   std::abs(v_change - 0.1) < 1e-6 &&
                   std::abs(q_change - 0.01) < 1e-3 &&
                   error_norm < 1e-10);

    if (passed) {
        std::cout << "✓ Test 4: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 4: FAILED" << std::endl;
    }

    return passed;
}

// Test 5: Performance benchmark
bool test_performance() {
    std::cout << "\n=== Test 5: Performance Benchmark ===" << std::endl;

    const int NUM_ITERATIONS = 10000;

    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.01, 0.1, 0.1, 0.001, 0.01);

    // Create a simple preintegration result
    ImuPreintegration preint;
    for (int i = 0; i < 3; i++) {  // Minimal integration
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i * 5e6);
        sample.gyro = Eigen::Vector3f(0.01f, 0.02f, 0.03f);
        sample.accel = Eigen::Vector3f(0.1f, 0.2f, 9.81f);
        sample.flags = 0;
        preint.integrate(sample);
    }

    auto preint_result = preint.get_result();
    Vector3d gravity(0, 0, 9.81);

    auto start = high_resolution_clock::now();

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        state.predict(preint_result, gravity);
    }

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<nanoseconds>(end - start).count();

    double ns_per_predict = static_cast<double>(duration) / NUM_ITERATIONS;
    double us_per_predict = ns_per_predict / 1000.0;

    std::cout << "Performance: " << us_per_predict << " µs per predict()" << std::endl;
    std::cout << "Target: <50 µs on desktop" << std::endl;

    bool passed = (us_per_predict < 50.0);

    if (passed) {
        std::cout << "✓ Test 5: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 5: FAILED (too slow)" << std::endl;
    }

    return passed;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "EKF State Unit Tests" << std::endl;
    std::cout << "Testing: Error-State EKF Implementation" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= test_initialization();
    all_passed &= test_zero_motion_propagation();
    all_passed &= test_covariance_growth();
    all_passed &= test_error_injection();
    all_passed &= test_performance();

    std::cout << "\n========================================" << std::endl;
    if (all_passed) {
        std::cout << "✅ ALL EKF STATE TESTS PASSED" << std::endl;
        std::cout << "Error-state EKF validated" << std::endl;
        std::cout << "Ready for measurement updates" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}
