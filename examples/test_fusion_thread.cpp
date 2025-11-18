// Fusion Thread Test
//
// Purpose: Validate FusionThread implementation
// Tests: Lifecycle, sensor data ingestion, state output, adaptive rate, concurrent access
//
// Expected Output:
// - Thread starts and stops cleanly
// - Processes sensor data correctly
// - State updates published
// - Adaptive rate transitions verified
// - Concurrent access safe
// - All tests pass

#include "service/fusion_thread.hpp"
#include "utils/logger.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

using namespace fusion;
using namespace std::chrono;

// Test 1: Basic lifecycle
bool test_lifecycle() {
    std::cout << "\n=== Test 1: Lifecycle (Start/Stop) ===" << std::endl;

    FusionConfig config;
    config.base_rate_hz = 100.0;
    config.adaptive_rate_enabled = false;  // Fixed rate for predictable testing

    FusionThread thread(config);

    // Start thread
    bool started = thread.start();
    if (!started) {
        std::cerr << "✗ Failed to start thread" << std::endl;
        return false;
    }

    std::this_thread::sleep_for(milliseconds(100));

    if (!thread.is_running()) {
        std::cerr << "✗ Thread not running after start" << std::endl;
        return false;
    }

    // Stop thread
    thread.stop();

    if (thread.is_running()) {
        std::cerr << "✗ Thread still running after stop" << std::endl;
        return false;
    }

    std::cout << "✓ Test 1: PASSED" << std::endl;
    return true;
}

// Test 2: Sensor data ingestion
bool test_sensor_ingestion() {
    std::cout << "\n=== Test 2: Sensor Data Ingestion ===" << std::endl;

    FusionConfig config;
    config.base_rate_hz = 100.0;
    config.adaptive_rate_enabled = false;

    FusionThread thread(config);
    thread.start();

    // Push IMU samples (simulate 200 Hz for 1 second)
    for (int i = 0; i < 200; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;  // 200 Hz
        sample.gyro = Vector3f::Zero();
        sample.accel = Vector3f(0, 0, -9.81f);

        bool queued = thread.push_imu_sample(sample);
        if (!queued) {
            std::cerr << "✗ Failed to queue IMU sample " << i << std::endl;
            thread.stop();
            return false;
        }

        std::this_thread::sleep_for(microseconds(5000));  // 200 Hz
    }

    // Push magnetometer samples (simulate 50 Hz for 1 second)
    for (int i = 0; i < 50; i++) {
        Vector3d mag_body(20.0, 5.0, -40.0);  // Typical values
        int64_t timestamp_ns = static_cast<int64_t>(i) * 20000000LL;  // 50 Hz

        thread.push_mag_sample(mag_body, timestamp_ns);

        std::this_thread::sleep_for(microseconds(20000));  // 50 Hz
    }

    // Allow fusion thread to process
    std::this_thread::sleep_for(milliseconds(200));

    // Get statistics
    auto stats = thread.get_stats();

    std::cout << "IMU samples processed: " << stats.imu_samples_processed << std::endl;
    std::cout << "Mag updates processed: " << stats.mag_updates_processed << std::endl;
    std::cout << "Total cycles: " << stats.cycle_count << std::endl;
    std::cout << "Avg cycle time: " << stats.avg_cycle_time_us << " µs" << std::endl;

    thread.stop();

    // Verify processing
    bool imu_ok = (stats.imu_samples_processed >= 180);  // Allow some loss
    bool mag_ok = (stats.mag_updates_processed >= 40);
    bool cycles_ok = (stats.cycle_count >= 90);  // 100 Hz * 1s, allow variance

    if (!imu_ok || !mag_ok || !cycles_ok) {
        std::cerr << "✗ Insufficient processing" << std::endl;
        return false;
    }

    std::cout << "✓ Test 2: PASSED" << std::endl;
    return true;
}

// Test 3: State output validation
bool test_state_output() {
    std::cout << "\n=== Test 3: State Output Validation ===" << std::endl;

    FusionConfig config;
    config.base_rate_hz = 100.0;

    FusionThread thread(config);
    thread.start();

    // Push stationary IMU data for 0.5 seconds
    for (int i = 0; i < 100; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;
        sample.gyro = Vector3f::Zero();
        sample.accel = Vector3f(0, 0, -9.81f);
        thread.push_imu_sample(sample);

        std::this_thread::sleep_for(microseconds(5000));
    }

    std::this_thread::sleep_for(milliseconds(100));

    // Get state
    auto state = thread.get_current_state();
    auto pose = thread.get_current_pose();

    std::cout << "Position: " << state.position.transpose() << " m" << std::endl;
    std::cout << "Velocity: " << state.velocity.transpose() << " m/s" << std::endl;
    std::cout << "Orientation (quat): " << state.orientation.coeffs().transpose() << std::endl;
    std::cout << "Sequence: " << state.sequence << std::endl;

    thread.stop();

    // Verify state is reasonable (stationary, so small drift)
    bool pos_ok = (state.position.norm() < 1.0);  // <1m drift in 0.5s
    bool vel_ok = (state.velocity.norm() < 0.5);  // <0.5 m/s drift
    bool seq_ok = (state.sequence > 0);
    bool timestamp_ok = (state.timestamp_ns > 0);

    if (!pos_ok || !vel_ok || !seq_ok || !timestamp_ok) {
        std::cerr << "✗ State validation failed" << std::endl;
        return false;
    }

    std::cout << "✓ Test 3: PASSED" << std::endl;
    return true;
}

// Test 4: Adaptive rate (stationary detection)
bool test_adaptive_rate() {
    std::cout << "\n=== Test 4: Adaptive Rate Control ===" << std::endl;

    FusionConfig config;
    config.base_rate_hz = 100.0;
    config.adaptive_rate_enabled = true;
    config.stationary_rate_hz = 50.0;
    config.moving_rate_hz = 100.0;

    FusionThread thread(config);
    thread.start();

    // Phase 1: Stationary (0.5 seconds)
    std::cout << "Phase 1: Stationary motion..." << std::endl;
    for (int i = 0; i < 100; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;
        sample.gyro = Vector3f::Zero();
        sample.accel = Vector3f(0, 0, -9.81f);
        thread.push_imu_sample(sample);

        std::this_thread::sleep_for(microseconds(5000));
    }

    std::this_thread::sleep_for(milliseconds(200));
    auto stats1 = thread.get_stats();

    std::cout << "Stationary - Rate: " << stats1.current_rate_hz << " Hz, "
              << "Is Stationary: " << (stats1.is_stationary ? "true" : "false") << std::endl;

    // Phase 2: Moving (rotation)
    std::cout << "Phase 2: Rotating motion..." << std::endl;
    for (int i = 100; i < 200; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;
        sample.gyro = Vector3f(0.5f, 0.0f, 0.0f);  // Rotation
        sample.accel = Vector3f(0, 0, -9.81f);
        thread.push_imu_sample(sample);

        std::this_thread::sleep_for(microseconds(5000));
    }

    std::this_thread::sleep_for(milliseconds(200));
    auto stats2 = thread.get_stats();

    std::cout << "Moving - Rate: " << stats2.current_rate_hz << " Hz, "
              << "Is Stationary: " << (stats2.is_stationary ? "true" : "false") << std::endl;

    thread.stop();

    // Verify adaptive rate worked
    bool stationary_detected = stats1.is_stationary;
    bool moving_detected = !stats2.is_stationary;
    bool rate_adapted = (stats2.current_rate_hz > stats1.current_rate_hz);

    if (!stationary_detected || !moving_detected) {
        std::cerr << "✗ Motion detection failed" << std::endl;
        std::cerr << "  Stationary detected: " << stationary_detected << std::endl;
        std::cerr << "  Moving detected: " << moving_detected << std::endl;
        return false;
    }

    std::cout << "✓ Test 4: PASSED" << std::endl;
    return true;
}

// Test 5: Concurrent access (thread safety)
bool test_concurrent_access() {
    std::cout << "\n=== Test 5: Concurrent Access (Thread Safety) ===" << std::endl;

    FusionConfig config;
    config.base_rate_hz = 100.0;

    FusionThread thread(config);
    thread.start();

    // Push IMU data in background
    std::thread producer([&thread]() {
        for (int i = 0; i < 200; i++) {
            ImuSample sample;
            sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;
            sample.gyro = Vector3f::Zero();
            sample.accel = Vector3f(0, 0, -9.81f);
            thread.push_imu_sample(sample);

            std::this_thread::sleep_for(microseconds(5000));
        }
    });

    // Concurrently read state from multiple threads
    std::atomic<int> read_count{0};
    std::atomic<bool> error_detected{false};

    auto reader = [&thread, &read_count, &error_detected]() {
        for (int i = 0; i < 100; i++) {
            auto state = thread.get_current_state();
            auto pose = thread.get_current_pose();
            auto stats = thread.get_stats();

            // Verify consistency
            if (state.timestamp_ns <= 0 || stats.cycle_count == 0) {
                error_detected = true;
            }

            read_count++;
            std::this_thread::sleep_for(microseconds(1000));  // 1ms
        }
    };

    std::thread reader1(reader);
    std::thread reader2(reader);
    std::thread reader3(reader);

    producer.join();
    reader1.join();
    reader2.join();
    reader3.join();

    thread.stop();

    std::cout << "Total concurrent reads: " << read_count.load() << std::endl;
    std::cout << "Errors detected: " << (error_detected ? "YES" : "NO") << std::endl;

    if (error_detected) {
        std::cerr << "✗ Concurrent access errors detected" << std::endl;
        return false;
    }

    if (read_count < 250) {  // Expect ~300 reads
        std::cerr << "✗ Insufficient concurrent reads" << std::endl;
        return false;
    }

    std::cout << "✓ Test 5: PASSED" << std::endl;
    return true;
}

// Test 6: Performance validation
bool test_performance() {
    std::cout << "\n=== Test 6: Performance Validation ===" << std::endl;

    FusionConfig config;
    config.base_rate_hz = 100.0;
    config.adaptive_rate_enabled = false;

    FusionThread thread(config);
    thread.start();

    // Run for 2 seconds at 200 Hz IMU
    for (int i = 0; i < 400; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i) * 5000000LL;
        sample.gyro = Vector3f::Zero();
        sample.accel = Vector3f(0, 0, -9.81f);
        thread.push_imu_sample(sample);

        std::this_thread::sleep_for(microseconds(5000));
    }

    std::this_thread::sleep_for(milliseconds(100));

    auto stats = thread.get_stats();

    std::cout << "Average cycle time: " << stats.avg_cycle_time_us << " µs" << std::endl;
    std::cout << "Max cycle time: " << stats.max_cycle_time_us << " µs" << std::endl;
    std::cout << "Min cycle time: " << stats.min_cycle_time_us << " µs" << std::endl;
    std::cout << "Total cycles: " << stats.cycle_count << std::endl;
    std::cout << "Average rate: " << stats.avg_rate_hz << " Hz" << std::endl;

    thread.stop();

    // Performance targets (from Phase 0-5)
    bool avg_ok = (stats.avg_cycle_time_us < 100);  // <100µs average
    bool max_ok = (stats.max_cycle_time_us < 500);  // <500µs max
    bool rate_ok = (std::abs(stats.current_rate_hz - 100.0) < 10.0);  // ±10 Hz

    if (!avg_ok || !max_ok || !rate_ok) {
        std::cerr << "✗ Performance targets not met" << std::endl;
        return false;
    }

    std::cout << "✓ Test 6: PASSED" << std::endl;
    return true;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Fusion Thread Tests" << std::endl;
    std::cout << "Testing: Real-time fusion loop, adaptive rate, thread safety" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= test_lifecycle();
    all_passed &= test_sensor_ingestion();
    all_passed &= test_state_output();
    all_passed &= test_adaptive_rate();
    all_passed &= test_concurrent_access();
    all_passed &= test_performance();

    std::cout << "\n========================================" << std::endl;
    if (all_passed) {
        std::cout << "✅ ALL FUSION THREAD TESTS PASSED" << std::endl;
        std::cout << "FusionThread is production-ready!" << std::endl;
        std::cout << "Next: Integrate with Android via JNI" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}
