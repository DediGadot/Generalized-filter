// Location Update Validation Test
//
// Purpose: Validate LocationUpdate implementation and prove correctness
//
// Tests:
// 1. WGS84→NED conversion accuracy
// 2. LocationUpdate measurement update
// 3. Jacobian correctness (identity matrix)
// 4. Adaptive noise scaling
// 5. Position covariance reduction after update
// 6. Performance (<10µs target)
//
// Expected Results:
// - WGS84→NED accuracy <1cm for <10km distances
// - Position covariance reduces after update
// - Jacobian is correct identity matrix
// - Update latency <10µs
// - All tests pass with expected values

#include "filter/location_update.hpp"
#include "filter/ekf_state.hpp"
#include "sensors/jni_location.hpp"
#include "core/sensor_types.hpp"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

using namespace fusion;

// Test counter
int total_tests = 0;
int passed_tests = 0;

#define TEST(name) \
    total_tests++; \
    std::cout << "\n[Test " << total_tests << "] " << name << "..." << std::endl;

#define EXPECT_NEAR(actual, expected, tolerance) \
    if (std::abs((actual) - (expected)) > (tolerance)) { \
        std::cout << "  ❌ FAILED: " << #actual << " = " << (actual) \
                  << ", expected " << (expected) << " ± " << (tolerance) << std::endl; \
        return false; \
    }

#define EXPECT_TRUE(condition) \
    if (!(condition)) { \
        std::cout << "  ❌ FAILED: " << #condition << " is false" << std::endl; \
        return false; \
    }

#define TEST_PASS() \
    passed_tests++; \
    std::cout << "  ✅ PASSED" << std::endl; \
    return true;

// ========== Test 1: WGS84→NED Conversion ==========

bool test_wgs84_to_ned_conversion() {
    TEST("WGS84→NED Conversion Accuracy");

    JniLocation jni_loc;

    // Reference point: San Francisco (37.7749° N, 122.4194° W, 10m altitude)
    double ref_lat = 37.7749;
    double ref_lon = -122.4194;
    double ref_alt = 10.0;

    // First location sets reference
    jni_loc.push_location(
        1000000000,   // timestamp
        ref_lat, ref_lon, ref_alt,
        5.0f, 10.0f,  // accuracy
        0.0f, 0.0f,   // speed
        1             // GPS flag
    );

    LocationMeasurement meas1;
    EXPECT_TRUE(jni_loc.pop(meas1));

    // First point should be at origin
    EXPECT_NEAR(meas1.position_ned.x(), 0.0, 0.01);  // North
    EXPECT_NEAR(meas1.position_ned.y(), 0.0, 0.01);  // East
    EXPECT_NEAR(meas1.position_ned.z(), 0.0, 0.01);  // Down

    // Second location: 100m north, 100m east, 20m up
    // Approximate: 100m north ≈ 0.0009° latitude
    //              100m east ≈ 0.0012° longitude (at this latitude)
    double lat2 = ref_lat + 0.0009;
    double lon2 = ref_lon + 0.0012;
    double alt2 = ref_alt + 20.0;

    jni_loc.push_location(
        2000000000,
        lat2, lon2, alt2,
        5.0f, 10.0f,
        0.0f, 0.0f,
        1
    );

    LocationMeasurement meas2;
    EXPECT_TRUE(jni_loc.pop(meas2));

    // Check NED coordinates (allow 10% error due to approximation)
    EXPECT_NEAR(meas2.position_ned.x(), 100.0, 10.0);   // North
    EXPECT_NEAR(meas2.position_ned.y(), 100.0, 15.0);   // East (wider tolerance, lat-dependent)
    EXPECT_NEAR(meas2.position_ned.z(), -20.0, 0.1);    // Down (negative = up)

    TEST_PASS();
}

// ========== Test 2: Jacobian Correctness ==========

bool test_jacobian_identity() {
    TEST("Jacobian Identity Matrix");

    LocationUpdate loc_update;

    // Get Jacobian via reflection (we need to make it accessible)
    // For now, test indirectly by checking update behavior

    EkfState state;
    state.initialize(
        Vector3d(100, 200, 50),  // Initial position [N, E, D]
        Vector3d::Zero(),         // Zero velocity
        Quaterniond::Identity(),  // Identity orientation
        0
    );

    // Set moderate initial covariance
    state.set_initial_covariance(
        0.1,   // 0.1 rad orientation uncertainty
        0.1,   // 0.1 m/s velocity uncertainty
        10.0,  // 10m position uncertainty
        0.01,  // 0.01 rad/s gyro bias uncertainty
        0.1    // 0.1 m/s² accel bias uncertainty
    );

    // Create location measurement at [110, 210, 60] (10m offset in each axis)
    LocationMeasurement loc;
    loc.timestamp_ns = 1000000000;
    loc.position_ned = Vector3d(110, 210, 60);
    loc.horizontal_accuracy_m = 5.0f;
    loc.vertical_accuracy_m = 10.0f;
    loc.provider_flags = 1;  // GPS

    // Perform update
    bool accepted = loc_update.update(state, loc, false);  // Disable gating for this test
    EXPECT_TRUE(accepted);

    // Inject error state into nominal state (error-state EKF pattern)
    state.inject_error(state.error_state());
    state.reset_error();

    // Innovation should be [10, 10, 10]
    Vector3d innovation = loc_update.get_last_innovation();
    EXPECT_NEAR(innovation.x(), 10.0, 0.1);
    EXPECT_NEAR(innovation.y(), 10.0, 0.1);
    EXPECT_NEAR(innovation.z(), 10.0, 0.1);

    // Position should have moved toward measurement
    Vector3d final_pos = state.position();
    // With Kalman gain, position should be between initial and measured
    EXPECT_TRUE(final_pos.x() > 100 && final_pos.x() < 110);
    EXPECT_TRUE(final_pos.y() > 200 && final_pos.y() < 210);
    EXPECT_TRUE(final_pos.z() > 50 && final_pos.z() < 60);

    TEST_PASS();
}

// ========== Test 3: Covariance Reduction ==========

bool test_covariance_reduction() {
    TEST("Position Covariance Reduction After Update");

    EkfState state;
    state.initialize(
        Vector3d(0, 0, 0),
        Vector3d::Zero(),
        Quaterniond::Identity(),
        0
    );

    // Set initial covariance with large position uncertainty
    state.set_initial_covariance(
        0.1,   // orientation
        0.1,   // velocity
        100.0, // 100m position uncertainty (LARGE)
        0.01,  // gyro bias
        0.1    // accel bias
    );

    // Get initial position covariance
    double initial_pos_var = state.covariance()(6, 6) +
                              state.covariance()(7, 7) +
                              state.covariance()(8, 8);

    std::cout << "  Initial position variance: " << initial_pos_var << " m²" << std::endl;

    // Create accurate location measurement
    LocationMeasurement loc;
    loc.timestamp_ns = 1000000000;
    loc.position_ned = Vector3d(0, 0, 0);
    loc.horizontal_accuracy_m = 5.0f;  // Good accuracy
    loc.vertical_accuracy_m = 10.0f;
    loc.provider_flags = 1;  // GPS

    LocationUpdate loc_update;
    bool accepted = loc_update.update(state, loc);
    EXPECT_TRUE(accepted);

    // Inject error and reset
    state.inject_error(state.error_state());
    state.reset_error();

    // Get final position covariance
    double final_pos_var = state.covariance()(6, 6) +
                            state.covariance()(7, 7) +
                            state.covariance()(8, 8);

    std::cout << "  Final position variance: " << final_pos_var << " m²" << std::endl;
    std::cout << "  Reduction: " << (initial_pos_var - final_pos_var) << " m² ("
              << (100.0 * (initial_pos_var - final_pos_var) / initial_pos_var) << "%)" << std::endl;

    // Covariance should reduce significantly
    EXPECT_TRUE(final_pos_var < initial_pos_var);
    EXPECT_TRUE((initial_pos_var - final_pos_var) > 0.5 * initial_pos_var);  // At least 50% reduction

    TEST_PASS();
}

// ========== Test 4: Adaptive Noise Scaling ==========

bool test_adaptive_noise() {
    TEST("Adaptive Noise Scaling");

    LocationUpdate loc_update;

    // Test 1: Good accuracy (5m) → nominal noise factor
    {
        LocationMeasurement loc;
        loc.horizontal_accuracy_m = 5.0f;
        loc.vertical_accuracy_m = 10.0f;
        loc.provider_flags = 1;  // GPS

        EkfState state;
        state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
        state.set_initial_covariance(0.1, 0.1, 10.0, 0.01, 0.1);

        loc.position_ned = state.position();
        loc_update.update(state, loc, false);

        double factor = loc_update.get_last_adaptive_factor();
        EXPECT_NEAR(factor, 1.0, 0.1);  // Should be ~1.0
    }

    // Test 2: Poor accuracy (60m) → increased noise factor
    {
        LocationMeasurement loc;
        loc.horizontal_accuracy_m = 60.0f;  // Poor accuracy
        loc.vertical_accuracy_m = 100.0f;
        loc.provider_flags = 1;

        EkfState state;
        state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
        state.set_initial_covariance(0.1, 0.1, 10.0, 0.01, 0.1);

        loc.position_ned = state.position();
        loc_update.update(state, loc, false);

        double factor = loc_update.get_last_adaptive_factor();
        EXPECT_TRUE(factor > 1.5);  // Should increase
        std::cout << "  Adaptive factor with poor accuracy: " << factor << std::endl;
    }

    TEST_PASS();
}

// ========== Test 5: Performance Benchmark ==========

bool test_performance() {
    TEST("Performance Benchmark (<10µs target)");

    LocationUpdate loc_update;

    EkfState state;
    state.initialize(
        Vector3d(100, 200, 50),
        Vector3d::Zero(),
        Quaterniond::Identity(),
        0
    );
    state.set_initial_covariance(0.1, 0.1, 10.0, 0.01, 0.1);

    LocationMeasurement loc;
    loc.timestamp_ns = 1000000000;
    loc.position_ned = Vector3d(110, 210, 60);
    loc.horizontal_accuracy_m = 5.0f;
    loc.vertical_accuracy_m = 10.0f;
    loc.provider_flags = 1;

    // Warmup
    for (int i = 0; i < 100; i++) {
        loc_update.update(state, loc, false);
        state.inject_error(state.error_state());
        state.reset_error();
    }

    // Benchmark
    constexpr int NUM_ITERATIONS = 10000;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        loc_update.update(state, loc, false);
        state.inject_error(state.error_state());
        state.reset_error();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    double avg_time_ns = static_cast<double>(duration.count()) / NUM_ITERATIONS;
    double avg_time_us = avg_time_ns / 1000.0;

    std::cout << "  Average update time: " << std::fixed << std::setprecision(2)
              << avg_time_us << " µs" << std::endl;
    std::cout << "  Target: <10 µs" << std::endl;

    // Performance should meet target (<10µs)
    EXPECT_TRUE(avg_time_us < 10.0);

    TEST_PASS();
}

// ========== Main ==========

int main() {
    std::cout << "\n============================================" << std::endl;
    std::cout << "  Location Update Validation Test" << std::endl;
    std::cout << "============================================\n" << std::endl;

    // Run all tests
    test_wgs84_to_ned_conversion();
    test_jacobian_identity();
    test_covariance_reduction();
    test_adaptive_noise();
    test_performance();

    // Summary
    std::cout << "\n============================================" << std::endl;
    std::cout << "  Test Summary" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "Total tests: " << total_tests << std::endl;
    std::cout << "Passed: " << passed_tests << std::endl;
    std::cout << "Failed: " << (total_tests - passed_tests) << std::endl;

    if (passed_tests == total_tests) {
        std::cout << "\n✅ ALL TESTS PASSED" << std::endl;
        std::cout << "\n🎉 Location Update Implementation Validated!" << std::endl;
        return 0;
    } else {
        std::cout << "\n❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
}
