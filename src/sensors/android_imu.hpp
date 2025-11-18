// Android IMU Sensor Wrapper using ASensorManager (NDK)
//
// Purpose: Direct access to IMU sensors (accelerometer + gyroscope) at hardware rate
// Documentation: https://developer.android.com/ndk/reference/group/sensor
//
// Key Features:
// - Direct NDK access via ASensorManager (bypasses Java layer)
// - Hardware-rate sampling (200 Hz target)
// - Low-latency callbacks via ALooper
// - Hardware timestamps (CLOCK_BOOTTIME)
// - Lock-free queue for thread-safe data transfer
//
// Sample Usage:
//   AndroidImu imu(callback_function);
//   imu.start(200.0f);  // 200 Hz
//   // ... sensor data arrives in callback ...
//   imu.stop();
//
// Expected Behavior:
//   - Accelerometer + gyroscope data at 200 Hz
//   - Hardware timestamps synchronized
//   - Data pushed to lock-free queue
//   - <1% CPU overhead
//
// Platform: Android API 24+ (Nougat), requires android.sensor permission

#pragma once

#ifdef __ANDROID__

#include <android/looper.h>
#include <android/sensor.h>
#include <atomic>
#include <functional>
#include <memory>

#include "core/lockfree_queue.hpp"
#include "core/sensor_types.hpp"

namespace fusion {

class AndroidImu {
public:
    using ImuCallback = std::function<void(const ImuSample&)>;

    // Constructor
    // queue_size: Power-of-2 capacity for lock-free queue (default 512)
    explicit AndroidImu(size_t queue_size = 512);

    ~AndroidImu();

    // Non-copyable, non-movable
    AndroidImu(const AndroidImu&) = delete;
    AndroidImu& operator=(const AndroidImu&) = delete;
    AndroidImu(AndroidImu&&) = delete;
    AndroidImu& operator=(AndroidImu&&) = delete;

    // Start IMU streaming
    // rate_hz: Target sampling rate in Hz (e.g., 200.0)
    // Returns: true on success, false on failure
    bool start(float rate_hz = 200.0f);

    // Stop IMU streaming
    void stop();

    // Check if streaming is active
    bool is_running() const { return running_.load(std::memory_order_acquire); }

    // Pop IMU sample from queue (non-blocking)
    // Returns: true if sample retrieved, false if queue empty
    bool pop(ImuSample& sample);

    // Get queue statistics
    size_t queue_size() const { return imu_queue_.size(); }
    size_t queue_capacity() const { return imu_queue_.capacity(); }

    // Get overflow counter (number of dropped samples)
    uint64_t get_overflow_count() const {
        return overflow_count_.load(std::memory_order_acquire);
    }

private:
    // Sensor callback (called by ALooper thread)
    static int sensor_callback(int fd, int events, void* data);

    // Process sensor events
    void process_events();

    // Sensor manager and queue
    ASensorManager* sensor_manager_;
    ASensorEventQueue* event_queue_;
    const ASensor* accel_sensor_;
    const ASensor* gyro_sensor_;
    ALooper* looper_;

    // Lock-free queue for data transfer
    LockFreeQueue<ImuSample> imu_queue_;

    // State tracking
    std::atomic<bool> running_;
    std::atomic<uint64_t> overflow_count_;

    // Buffered IMU data (waiting for both accel + gyro)
    struct ImuBuffer {
        int64_t timestamp_ns;
        Eigen::Vector3f accel;
        Eigen::Vector3f gyro;
        bool has_accel;
        bool has_gyro;
    };
    ImuBuffer buffer_;
};

}  // namespace fusion

#else
// Stub for non-Android platforms (desktop testing)
namespace fusion {
class AndroidImu {
public:
    explicit AndroidImu(size_t = 512) {}
    bool start(float = 200.0f) { return false; }
    void stop() {}
    bool is_running() const { return false; }
    bool pop(ImuSample&) { return false; }
    size_t queue_size() const { return 0; }
    size_t queue_capacity() const { return 0; }
    uint64_t get_overflow_count() const { return 0; }
};
}  // namespace fusion
#endif  // __ANDROID__
