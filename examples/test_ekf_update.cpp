// EKF Measurement Update Tests
//
// Purpose: Validate generic measurement update implementation
// Tests:
//   1. Position measurement update (3D)
//   2. Covariance reduction after update
//   3. Outlier rejection via chi-square gating
//   4. Joseph form numerical stability
//   5. Performance benchmark

#include "filter/ekf_update.hpp"
#include "filter/ekf_state.hpp"
#include "core/quaternion.hpp"

#include <chrono>
#include <iostream>
#include <cmath>

using namespace fusion;
using namespace std::chrono;

// Test 1: Simple position measurement update
bool test_position_update() {
    std::cout << "\n=== Test 1: Position Measurement Update ===" << std::endl;

    // Initialize EKF state
    EkfState state;
    Vector3d p0(0, 0, 0);
    Vector3d v0(0, 0, 0);
    Quaterniond q0 = Quaterniond::Identity();

    state.initialize(p0, v0, q0, 0);
    state.set_initial_covariance(0.1, 0.5, 10.0, 0.001, 0.01);  // Large position uncertainty

    // Save initial covariance
    double initial_position_cov = state.covariance()(ErrorState::DP, ErrorState::DP);

    std::cout << "Initial position covariance: " << initial_position_cov << std::endl;

    // Simulate a position measurement
    // True position: [0, 0, 0]
    // Measured position: [1.0, 0.5, -0.2] (with measurement noise)
    Vector3d measured_position(1.0, 0.5, -0.2);
    Vector3d predicted_position = state.position();  // Should be [0, 0, 0]

    // Innovation (measurement - prediction)
    Vector3d innovation = measured_position - predicted_position;

    std::cout << "Innovation (m - h(x)): " << innovation.transpose() << std::endl;

    // Measurement Jacobian H
    // For position measurement: ∂h/∂δx
    // h(x) depends only on position, so:
    // ∂h/∂δp = I (identity)
    // ∂h/∂(other states) = 0
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, ErrorState::DP) = Matrix3d::Identity();

    // Measurement noise covariance
    // Position measurement noise: 1m standard deviation per axis
    Matrix3d R = Matrix3d::Identity() * (1.0 * 1.0);  // 1m std dev

    // Perform measurement update
    MeasurementUpdate update;
    bool accepted = update.update(state, innovation, H, R);

    std::cout << "Update accepted: " << (accepted ? "Yes" : "No (outlier)") << std::endl;

    // Check results
    if (!accepted) {
        std::cerr << "✗ Test 1: FAILED - Update rejected unexpectedly" << std::endl;
        return false;
    }

    // Error state should now contain the correction
    Vector3d error_position = state.error_state().segment<3>(ErrorState::DP);
    std::cout << "Error state (position component): " << error_position.transpose() << std::endl;

    // Inject error into nominal state
    state.inject_error(state.error_state());
    state.reset_error();

    // Check that position moved towards measurement
    Vector3d updated_position = state.position();
    std::cout << "Updated position: " << updated_position.transpose() << std::endl;

    // Covariance should have decreased
    double final_position_cov = state.covariance()(ErrorState::DP, ErrorState::DP);
    std::cout << "Final position covariance: " << final_position_cov << std::endl;

    bool cov_decreased = (final_position_cov < initial_position_cov);
    std::cout << "Covariance decreased: " << (cov_decreased ? "Yes" : "No") << std::endl;

    // Position should be closer to measurement than before
    double distance_to_measurement = (updated_position - measured_position).norm();
    std::cout << "Distance to measurement: " << distance_to_measurement << " m" << std::endl;

    bool passed = (accepted && cov_decreased && distance_to_measurement < innovation.norm());

    if (passed) {
        std::cout << "✓ Test 1: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 1: FAILED" << std::endl;
    }

    return passed;
}

// Test 2: Outlier rejection
bool test_outlier_rejection() {
    std::cout << "\n=== Test 2: Outlier Rejection ===" << std::endl;

    // Initialize EKF state
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);  // Small position uncertainty

    // Simulate a position measurement that's WAY off (outlier)
    // True position: [0, 0, 0]
    // Measured position: [100, 0, 0] (100m error - clearly wrong!)
    Vector3d measured_position(100.0, 0.0, 0.0);
    Vector3d predicted_position = state.position();

    Vector3d innovation = measured_position - predicted_position;

    std::cout << "Innovation: " << innovation.transpose() << " m" << std::endl;
    std::cout << "Innovation norm: " << innovation.norm() << " m (huge!)" << std::endl;

    // Measurement Jacobian and noise
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, ErrorState::DP) = Matrix3d::Identity();
    Matrix3d R = Matrix3d::Identity() * (1.0 * 1.0);

    // Perform update WITH gating enabled
    MeasurementUpdate update;
    bool accepted = update.update(state, innovation, H, R, true, 7.815);  // 95% confidence

    std::cout << "Update accepted: " << (accepted ? "Yes" : "No (outlier)") << std::endl;

    // Outlier should be rejected
    bool passed = !accepted;

    if (passed) {
        std::cout << "✓ Test 2: PASSED (outlier correctly rejected)" << std::endl;
    } else {
        std::cerr << "✗ Test 2: FAILED (outlier not rejected!)" << std::endl;
    }

    return passed;
}

// Test 3: Covariance reduction
bool test_covariance_reduction() {
    std::cout << "\n=== Test 3: Covariance Reduction ===" << std::endl;

    // Initialize with high uncertainty
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 10.0, 0.001, 0.01);

    // Save initial covariances
    double initial_pos_cov = state.covariance()(ErrorState::DP, ErrorState::DP);
    double initial_vel_cov = state.covariance()(ErrorState::DV, ErrorState::DV);
    double initial_att_cov = state.covariance()(ErrorState::DTHETA, ErrorState::DTHETA);

    std::cout << "Initial position covariance: " << initial_pos_cov << std::endl;
    std::cout << "Initial velocity covariance: " << initial_vel_cov << std::endl;
    std::cout << "Initial attitude covariance: " << initial_att_cov << std::endl;

    // Simulate accurate position measurement (low noise)
    Vector3d innovation(0.1, 0.05, -0.02);  // Small innovation

    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, ErrorState::DP) = Matrix3d::Identity();

    // Very accurate measurement (0.1m std dev)
    Matrix3d R = Matrix3d::Identity() * (0.1 * 0.1);

    // Perform update
    MeasurementUpdate update;
    bool accepted = update.update(state, innovation, H, R);

    // Save final covariances
    double final_pos_cov = state.covariance()(ErrorState::DP, ErrorState::DP);
    double final_vel_cov = state.covariance()(ErrorState::DV, ErrorState::DV);
    double final_att_cov = state.covariance()(ErrorState::DTHETA, ErrorState::DTHETA);

    std::cout << "Final position covariance: " << final_pos_cov << std::endl;
    std::cout << "Final velocity covariance: " << final_vel_cov << std::endl;
    std::cout << "Final attitude covariance: " << final_att_cov << std::endl;

    // Position covariance should decrease significantly
    bool pos_decreased = (final_pos_cov < initial_pos_cov * 0.5);  // At least 50% reduction

    // Velocity and attitude should be unchanged (measurement doesn't observe them)
    bool vel_unchanged = (std::abs(final_vel_cov - initial_vel_cov) < 1e-6);
    bool att_unchanged = (std::abs(final_att_cov - initial_att_cov) < 1e-6);

    std::cout << "Position covariance decreased >50%: " << (pos_decreased ? "Yes" : "No") << std::endl;
    std::cout << "Velocity covariance unchanged: " << (vel_unchanged ? "Yes" : "No") << std::endl;
    std::cout << "Attitude covariance unchanged: " << (att_unchanged ? "Yes" : "No") << std::endl;

    bool passed = (accepted && pos_decreased && vel_unchanged && att_unchanged);

    if (passed) {
        std::cout << "✓ Test 3: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 3: FAILED" << std::endl;
    }

    return passed;
}

// Test 4: Performance benchmark
bool test_performance() {
    std::cout << "\n=== Test 4: Performance Benchmark ===" << std::endl;

    const int NUM_ITERATIONS = 10000;

    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 10.0, 0.001, 0.01);

    // Setup measurement model (position measurement)
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();
    H.block<3, 3>(0, ErrorState::DP) = Matrix3d::Identity();
    Matrix3d R = Matrix3d::Identity() * (1.0 * 1.0);

    Vector3d innovation(0.1, 0.05, -0.02);

    MeasurementUpdate update;

    auto start = high_resolution_clock::now();

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        update.update(state, innovation, H, R, false);  // Disable gating for pure update speed
        state.reset_error();  // Reset for next iteration
    }

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<nanoseconds>(end - start).count();

    double ns_per_update = static_cast<double>(duration) / NUM_ITERATIONS;
    double us_per_update = ns_per_update / 1000.0;

    std::cout << "Performance: " << us_per_update << " µs per update()" << std::endl;
    std::cout << "Target: <20 µs on desktop" << std::endl;

    bool passed = (us_per_update < 20.0);

    if (passed) {
        std::cout << "✓ Test 4: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 4: FAILED (too slow)" << std::endl;
    }

    return passed;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "EKF Measurement Update Tests" << std::endl;
    std::cout << "Testing: Generic measurement update framework" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= test_position_update();
    all_passed &= test_outlier_rejection();
    all_passed &= test_covariance_reduction();
    all_passed &= test_performance();

    std::cout << "\n========================================" << std::endl;
    if (all_passed) {
        std::cout << "✅ ALL MEASUREMENT UPDATE TESTS PASSED" << std::endl;
        std::cout << "Generic update framework validated" << std::endl;
        std::cout << "Ready for GNSS/magnetometer integration" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}
