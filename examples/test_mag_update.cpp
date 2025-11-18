// Magnetometer Update Tests
//
// Purpose: Validate magnetometer measurement update implementation
// Tests:
//   1. World Magnetic Model (WMM) reference field
//   2. Basic magnetometer update (heading correction)
//   3. Adaptive noise for magnetic disturbances
//   4. Outlier rejection via chi-square gating
//   5. Performance benchmark

#include "filter/mag_update.hpp"
#include "filter/ekf_state.hpp"
#include "core/quaternion.hpp"

#include <chrono>
#include <iostream>
#include <cmath>

using namespace fusion;
using namespace std::chrono;

// Test 1: World Magnetic Model reference field
bool test_wmm_reference_field() {
    std::cout << "\n=== Test 1: World Magnetic Model Reference Field ===" << std::endl;

    WorldMagneticModel wmm;

    // Test location: San Francisco (37.7749° N, 122.4194° W, 100m altitude)
    wmm.set_location(37.7749, -122.4194, 100.0);

    // Get reference field
    Vector3d mag_ned = wmm.get_reference_field_ned();

    std::cout << "Location: San Francisco (37.7749° N, 122.4194° W, 100m)" << std::endl;
    std::cout << "Magnetic field (NED): " << mag_ned.transpose() << " μT" << std::endl;
    std::cout << "Field strength: " << wmm.get_field_strength() << " μT" << std::endl;
    std::cout << "Declination: " << wmm.get_declination() * 180.0 / M_PI << "°" << std::endl;
    std::cout << "Inclination: " << wmm.get_inclination() * 180.0 / M_PI << "°" << std::endl;

    // Verify field strength is reasonable
    double field_strength = mag_ned.norm();
    bool strength_ok = (field_strength > 20.0 && field_strength < 70.0);  // Typical range: 25-65 μT

    std::cout << "Field strength in range [20, 70] μT: " << (strength_ok ? "Yes" : "No") << std::endl;

    // Verify declination is reasonable
    double declination_deg = wmm.get_declination() * 180.0 / M_PI;
    bool declination_ok = (std::abs(declination_deg) < 30.0);  // Should be within ±30°

    std::cout << "Declination in range [-30°, 30°]: " << (declination_ok ? "Yes" : "No") << std::endl;

    // Verify inclination is reasonable for Northern Hemisphere
    double inclination_deg = wmm.get_inclination() * 180.0 / M_PI;
    bool inclination_ok = (inclination_deg > 30.0 && inclination_deg < 90.0);  // Northern hemisphere

    std::cout << "Inclination in range [30°, 90°] (Northern Hemisphere): " << (inclination_ok ? "Yes" : "No") << std::endl;

    bool passed = (strength_ok && declination_ok && inclination_ok);

    if (passed) {
        std::cout << "✓ Test 1: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 1: FAILED" << std::endl;
    }

    return passed;
}

// Test 2: Basic magnetometer update (heading correction)
bool test_magnetometer_update() {
    std::cout << "\n=== Test 2: Magnetometer Update (Heading Correction) ===" << std::endl;

    // Initialize EKF state with yaw error
    EkfState state;

    // Start with zero rotation (aligned with navigation frame)
    Quaterniond q0 = Quaterniond::Identity();
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), q0, 0);
    state.set_initial_covariance(0.5, 0.5, 1.0, 0.001, 0.01);  // 0.5 rad attitude uncertainty

    // Save initial yaw covariance
    double initial_att_cov = state.covariance()(ErrorState::DTHETA, ErrorState::DTHETA);
    std::cout << "Initial attitude covariance: " << initial_att_cov << std::endl;

    // Setup magnetometer update
    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);  // San Francisco

    // Get reference field
    Vector3d mag_nav_reference = mag_update.get_reference_field();
    std::cout << "Reference field (NED): " << mag_nav_reference.transpose() << " μT" << std::endl;

    // Simulate perfect measurement (no rotation, so body = nav frame)
    // Add small noise to make it realistic
    Vector3d mag_body_measured = mag_nav_reference + Vector3d(0.1, 0.05, -0.02);

    std::cout << "Measured field (body): " << mag_body_measured.transpose() << " μT" << std::endl;

    // Perform update
    bool accepted = mag_update.update(state, mag_body_measured);

    std::cout << "Update accepted: " << (accepted ? "Yes" : "No") << std::endl;

    if (!accepted) {
        std::cerr << "✗ Test 2: FAILED - Update rejected unexpectedly" << std::endl;
        return false;
    }

    // Check innovation
    Vector3d innovation = mag_update.get_last_innovation();
    std::cout << "Innovation: " << innovation.transpose() << " μT" << std::endl;

    // Inject error into nominal state
    state.inject_error(state.error_state());
    state.reset_error();

    // Check covariance reduction
    double final_att_cov = state.covariance()(ErrorState::DTHETA, ErrorState::DTHETA);
    std::cout << "Final attitude covariance: " << final_att_cov << std::endl;

    bool cov_decreased = (final_att_cov < initial_att_cov);
    std::cout << "Covariance decreased: " << (cov_decreased ? "Yes" : "No") << std::endl;

    bool passed = (accepted && cov_decreased);

    if (passed) {
        std::cout << "✓ Test 2: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 2: FAILED" << std::endl;
    }

    return passed;
}

// Test 3: Adaptive noise for magnetic disturbances
bool test_adaptive_noise() {
    std::cout << "\n=== Test 3: Adaptive Noise (Magnetic Disturbance) ===" << std::endl;

    // Initialize EKF state
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

    // Setup magnetometer update
    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);

    // Get reference field
    Vector3d mag_nav_reference = mag_update.get_reference_field();
    double reference_norm = mag_nav_reference.norm();

    std::cout << "Reference field strength: " << reference_norm << " μT" << std::endl;

    // === Test 3a: Clean measurement (no disturbance) ===
    std::cout << "\nTest 3a: Clean measurement" << std::endl;
    Vector3d mag_clean = mag_nav_reference + Vector3d(0.1, 0.05, -0.02);  // Small noise

    mag_update.update(state, mag_clean, false);  // Disable gating for this test
    double clean_factor = mag_update.get_last_adaptive_factor();

    std::cout << "Clean measurement adaptive factor: " << clean_factor << std::endl;

    bool clean_factor_ok = (clean_factor < 1.5);  // Should be close to 1.0
    std::cout << "Adaptive factor close to 1.0: " << (clean_factor_ok ? "Yes" : "No") << std::endl;

    // === Test 3b: Disturbed measurement (large field deviation) ===
    std::cout << "\nTest 3b: Disturbed measurement (2x field strength)" << std::endl;

    // Simulate magnetic disturbance: field strength doubled (e.g., near metal)
    Vector3d mag_disturbed = 2.0 * mag_nav_reference;

    bool is_disturbance = mag_update.is_magnetic_disturbance(mag_disturbed);
    std::cout << "Magnetic disturbance detected: " << (is_disturbance ? "Yes" : "No") << std::endl;

    mag_update.update(state, mag_disturbed, false);  // Disable gating for this test
    double disturbed_factor = mag_update.get_last_adaptive_factor();

    std::cout << "Disturbed measurement adaptive factor: " << disturbed_factor << std::endl;

    bool disturbed_factor_ok = (disturbed_factor > 3.0);  // Should be significantly higher
    std::cout << "Adaptive factor increased: " << (disturbed_factor_ok ? "Yes" : "No") << std::endl;

    bool passed = (clean_factor_ok && is_disturbance && disturbed_factor_ok);

    if (passed) {
        std::cout << "✓ Test 3: PASSED (adaptive noise working correctly)" << std::endl;
    } else {
        std::cerr << "✗ Test 3: FAILED" << std::endl;
    }

    return passed;
}

// Test 4: Outlier rejection
bool test_outlier_rejection() {
    std::cout << "\n=== Test 4: Outlier Rejection ===" << std::endl;

    // Initialize EKF state
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);  // Low uncertainty

    // Setup magnetometer update
    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);

    // Get reference field
    Vector3d mag_nav_reference = mag_update.get_reference_field();

    // Simulate outlier measurement (completely wrong direction)
    Vector3d mag_outlier = -mag_nav_reference;  // Opposite direction!

    std::cout << "Reference field: " << mag_nav_reference.transpose() << " μT" << std::endl;
    std::cout << "Outlier field: " << mag_outlier.transpose() << " μT (opposite direction!)" << std::endl;

    // Perform update WITH gating enabled
    bool accepted = mag_update.update(state, mag_outlier, true);

    std::cout << "Update accepted: " << (accepted ? "Yes" : "No (outlier)") << std::endl;

    // Outlier should be rejected
    bool passed = !accepted;

    if (passed) {
        std::cout << "✓ Test 4: PASSED (outlier correctly rejected)" << std::endl;
    } else {
        std::cerr << "✗ Test 4: FAILED (outlier not rejected!)" << std::endl;
    }

    return passed;
}

// Test 5: Performance benchmark
bool test_performance() {
    std::cout << "\n=== Test 5: Performance Benchmark ===" << std::endl;

    const int NUM_ITERATIONS = 10000;

    // Initialize EKF state
    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

    // Setup magnetometer update
    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);

    // Get reference field
    Vector3d mag_nav_reference = mag_update.get_reference_field();

    // Simulated measurement (with small noise)
    Vector3d mag_body_measured = mag_nav_reference + Vector3d(0.1, 0.05, -0.02);

    auto start = high_resolution_clock::now();

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        mag_update.update(state, mag_body_measured, false);  // Disable gating for pure update speed
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
        std::cout << "✓ Test 5: PASSED" << std::endl;
    } else {
        std::cerr << "✗ Test 5: FAILED (too slow)" << std::endl;
    }

    return passed;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Magnetometer Update Tests" << std::endl;
    std::cout << "Testing: Magnetometer heading correction with WMM" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= test_wmm_reference_field();
    all_passed &= test_magnetometer_update();
    all_passed &= test_adaptive_noise();
    all_passed &= test_outlier_rejection();
    all_passed &= test_performance();

    std::cout << "\n========================================" << std::endl;
    if (all_passed) {
        std::cout << "✅ ALL MAGNETOMETER UPDATE TESTS PASSED" << std::endl;
        std::cout << "Magnetometer integration validated" << std::endl;
        std::cout << "Ready for multi-sensor fusion" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}
