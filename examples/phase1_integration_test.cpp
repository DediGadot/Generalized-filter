// Phase 1 Integration Test
//
// Purpose: Validate all Phase 1 components work together
// Tests:
//   1. Lock-free queue performance with sensor data types
//   2. Simulated sensor data flow (IMU, GNSS, MAG)
//   3. Multi-threaded producer-consumer pattern
//   4. Queue overflow handling
//
// This test runs on desktop and simulates the Android sensor pipeline

#include "core/lockfree_queue.hpp"
#include "core/sensor_types.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>

using namespace fusion;
using namespace std::chrono;

// Test 1: Lock-free queue with actual sensor types
bool test_queue_with_sensor_types() {
    std::cout << "\n=== Test 1: Queue with Sensor Types ===" << std::endl;

    // Test with ImuSample
    {
        LockFreeQueue<ImuSample> imu_queue(512);

        ImuSample sample;
        sample.timestamp_ns = 1000000000;
        sample.gyro << 0.1f, 0.2f, 0.3f;
        sample.accel << 0.0f, 0.0f, 9.81f;
        sample.flags = 0;

        if (!imu_queue.push(sample)) {
            std::cerr << "FAIL: Failed to push IMU sample" << std::endl;
            return false;
        }

        ImuSample retrieved;
        if (!imu_queue.pop(retrieved)) {
            std::cerr << "FAIL: Failed to pop IMU sample" << std::endl;
            return false;
        }

        if ((retrieved.gyro - sample.gyro).norm() > 1e-6) {
            std::cerr << "FAIL: IMU data mismatch" << std::endl;
            return false;
        }

        std::cout << "✓ ImuSample queue: PASSED" << std::endl;
    }

    // Test with GnssMeasurement
    {
        LockFreeQueue<GnssMeasurement> gnss_queue(16);

        GnssMeasurement meas;
        meas.timestamp_ns = 2000000000;
        meas.num_sats = 8;
        for (int i = 0; i < 8; i++) {
            meas.sats[i].prn = i + 1;
            meas.sats[i].pseudorange = 20000000.0 + i * 1000.0;
            meas.sats[i].doppler = 10.0f;
            meas.sats[i].cn0 = 40.0f;
        }

        if (!gnss_queue.push(meas)) {
            std::cerr << "FAIL: Failed to push GNSS measurement" << std::endl;
            return false;
        }

        GnssMeasurement retrieved;
        if (!gnss_queue.pop(retrieved)) {
            std::cerr << "FAIL: Failed to pop GNSS measurement" << std::endl;
            return false;
        }

        if (retrieved.num_sats != 8) {
            std::cerr << "FAIL: GNSS data mismatch" << std::endl;
            return false;
        }

        std::cout << "✓ GnssMeasurement queue: PASSED" << std::endl;
    }

    // Test with MagMeasurement
    {
        LockFreeQueue<MagMeasurement> mag_queue(64);

        MagMeasurement sample;
        sample.timestamp_ns = 3000000000;
        sample.mag << 25.0f, 0.0f, 45.0f;  // Typical magnetic field
        sample.temperature = 25.0f;

        if (!mag_queue.push(sample)) {
            std::cerr << "FAIL: Failed to push MAG sample" << std::endl;
            return false;
        }

        MagMeasurement retrieved;
        if (!mag_queue.pop(retrieved)) {
            std::cerr << "FAIL: Failed to pop MAG sample" << std::endl;
            return false;
        }

        if ((retrieved.mag - sample.mag).norm() > 1e-6) {
            std::cerr << "FAIL: MAG data mismatch" << std::endl;
            return false;
        }

        std::cout << "✓ MagMeasurement queue: PASSED" << std::endl;
    }

    return true;
}

// Test 2: Simulated sensor pipeline (multi-threaded)
bool test_simulated_sensor_pipeline() {
    std::cout << "\n=== Test 2: Simulated Sensor Pipeline ===" << std::endl;

    LockFreeQueue<ImuSample> imu_queue(512);
    std::atomic<bool> producer_done{false};
    std::atomic<int> samples_produced{0};
    std::atomic<int> samples_consumed{0};

    const int TARGET_SAMPLES = 200;  // Simulate 1 second at 200 Hz (reduced for faster test)

    // Producer thread (simulates IMU sensor)
    std::thread producer([&]() {
        auto start_time = steady_clock::now();

        for (int i = 0; i < TARGET_SAMPLES; i++) {
            ImuSample sample;
            sample.timestamp_ns = static_cast<int64_t>(i) * 5000000;  // 5ms spacing
            sample.gyro << 0.01f, 0.02f, 0.03f;
            sample.accel << 0.0f, 0.0f, 9.81f;
            sample.flags = 0;

            while (!imu_queue.push(sample)) {
                std::this_thread::yield();
            }

            samples_produced.fetch_add(1);

            // Simulate 200 Hz timing (5ms period) - reduced to 100µs for faster test
            std::this_thread::sleep_for(microseconds(100));
        }

        producer_done.store(true);

        auto end_time = steady_clock::now();
        auto duration = duration_cast<milliseconds>(end_time - start_time).count();
        std::cout << "Producer: Generated " << TARGET_SAMPLES << " samples in "
                  << duration << " ms" << std::endl;
    });

    // Consumer thread (simulates fusion filter)
    std::thread consumer([&]() {
        while (!producer_done.load() || !imu_queue.empty()) {
            ImuSample sample;
            if (imu_queue.pop(sample)) {
                samples_consumed.fetch_add(1);

                // Simulate processing (reduced for faster test)
                std::this_thread::sleep_for(microseconds(50));
            } else {
                std::this_thread::yield();
            }
        }

        std::cout << "Consumer: Processed " << samples_consumed.load() << " samples"
                  << std::endl;
    });

    producer.join();
    consumer.join();

    if (samples_consumed.load() != TARGET_SAMPLES) {
        std::cerr << "FAIL: Sample count mismatch (produced: " << TARGET_SAMPLES
                  << ", consumed: " << samples_consumed.load() << ")" << std::endl;
        return false;
    }

    std::cout << "✓ Simulated pipeline: PASSED (" << TARGET_SAMPLES << " samples)"
              << std::endl;

    return true;
}

// Test 3: Queue overflow handling
bool test_queue_overflow() {
    std::cout << "\n=== Test 3: Queue Overflow Handling ===" << std::endl;

    LockFreeQueue<ImuSample> small_queue(16);  // Small queue to force overflow

    int overflow_count = 0;

    // Try to push more than capacity
    for (int i = 0; i < 100; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<int64_t>(i) * 1000000;
        sample.gyro << 0.1f, 0.2f, 0.3f;
        sample.accel << 0.0f, 0.0f, 9.81f;
        sample.flags = 0;

        if (!small_queue.push(sample)) {
            overflow_count++;
        }
    }

    if (overflow_count == 0) {
        std::cerr << "FAIL: Expected overflows but got none" << std::endl;
        return false;
    }

    std::cout << "✓ Overflow handling: PASSED (" << overflow_count << " overflows detected)"
              << std::endl;

    return true;
}

// Test 4: Performance benchmark (all sensor types)
bool test_performance_benchmark() {
    std::cout << "\n=== Test 4: Performance Benchmark ===" << std::endl;

    const int ITERATIONS = 100000;

    // IMU throughput
    {
        LockFreeQueue<ImuSample> queue(1024);

        auto start = high_resolution_clock::now();

        for (int i = 0; i < ITERATIONS; i++) {
            ImuSample sample;
            sample.timestamp_ns = static_cast<int64_t>(i) * 1000000;
            sample.gyro << 0.1f, 0.2f, 0.3f;
            sample.accel << 0.0f, 0.0f, 9.81f;
            sample.flags = 0;

            queue.push(sample);

            ImuSample retrieved;
            queue.pop(retrieved);
        }

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<nanoseconds>(end - start).count();
        double ns_per_op = static_cast<double>(duration) / (2.0 * ITERATIONS);

        std::cout << "✓ IMU throughput: " << ns_per_op << " ns/operation" << std::endl;
    }

    // GNSS throughput
    {
        LockFreeQueue<GnssMeasurement> queue(16);

        auto start = high_resolution_clock::now();

        for (int i = 0; i < 1000; i++) {  // Fewer iterations (GNSS is heavier)
            GnssMeasurement meas;
            meas.timestamp_ns = static_cast<int64_t>(i) * 1000000000;
            meas.num_sats = 8;
            for (int j = 0; j < 8; j++) {
                meas.sats[j].prn = j + 1;
                meas.sats[j].pseudorange = 20000000.0;
                meas.sats[j].doppler = 10.0f;
                meas.sats[j].cn0 = 40.0f;
            }

            queue.push(meas);

            GnssMeasurement retrieved;
            queue.pop(retrieved);
        }

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<nanoseconds>(end - start).count();
        double ns_per_op = static_cast<double>(duration) / 2000.0;

        std::cout << "✓ GNSS throughput: " << ns_per_op << " ns/operation" << std::endl;
    }

    return true;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Phase 1 Integration Test" << std::endl;
    std::cout << "Testing: Lock-free queues + Sensor pipeline" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= test_queue_with_sensor_types();
    all_passed &= test_simulated_sensor_pipeline();
    all_passed &= test_queue_overflow();
    all_passed &= test_performance_benchmark();

    std::cout << "\n========================================" << std::endl;
    if (all_passed) {
        std::cout << "✅ ALL PHASE 1 TESTS PASSED" << std::endl;
        std::cout << "Lock-free queues validated" << std::endl;
        std::cout << "Sensor pipeline architecture validated" << std::endl;
        std::cout << "Ready for Phase 2: IMU Preintegration" << std::endl;
    } else {
        std::cout << "❌ SOME TESTS FAILED" << std::endl;
        return 1;
    }
    std::cout << "========================================" << std::endl;

    return 0;
}
