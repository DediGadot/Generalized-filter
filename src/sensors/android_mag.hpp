// Android Magnetometer Sensor Wrapper
//
// Purpose: Access magnetometer sensor for heading estimation
// Documentation: https://developer.android.com/ndk/reference/group/sensor
//
// Key Features:
// - Magnetometer sampling at 50 Hz (lower than IMU)
// - Hard iron / soft iron calibration support
// - Temperature compensation
// - Lock-free queue for thread-safe data transfer
//
// Sample Usage:
//   AndroidMag mag;
//   mag.start(50.0f);  // 50 Hz
//   MagSample sample;
//   if (mag.pop(sample)) {
//       // Process magnetometer data
//   }
//   mag.stop();
//
// Expected Behavior:
//   - Magnetic field data at 50 Hz
//   - Raw 3-axis measurements in µT (microtesla)
//   - Temperature readings if available
//
// Platform: Android API 24+

#pragma once

#ifdef __ANDROID__

#include <android/looper.h>
#include <android/sensor.h>
#include <atomic>
#include <memory>

#include "core/lockfree_queue.hpp"
#include "core/sensor_types.hpp"

namespace fusion {

// Magnetometer queue stores MagMeasurement samples; keep alias for clarity.
using MagSample = MagMeasurement;

class AndroidMag {
public:
    // Constructor
    // queue_size: Power-of-2 capacity (default 64)
    explicit AndroidMag(size_t queue_size = 64);

    ~AndroidMag();

    // Non-copyable, non-movable
    AndroidMag(const AndroidMag&) = delete;
    AndroidMag& operator=(const AndroidMag&) = delete;
    AndroidMag(AndroidMag&&) = delete;
    AndroidMag& operator=(AndroidMag&&) = delete;

    // Start magnetometer streaming
    // rate_hz: Target sampling rate in Hz (e.g., 50.0)
    // Returns: true on success, false on failure
    bool start(float rate_hz = 50.0f);

    // Stop streaming
    void stop();

    // Check if streaming is active
    bool is_running() const { return running_.load(std::memory_order_acquire); }

    // Pop magnetometer sample from queue (non-blocking)
    // Returns: true if sample retrieved, false if queue empty
    bool pop(MagSample& sample);

    // Get queue statistics
    size_t queue_size() const { return mag_queue_.size(); }
    size_t queue_capacity() const { return mag_queue_.capacity(); }

    // Get overflow counter
    uint64_t get_overflow_count() const {
        return overflow_count_.load(std::memory_order_acquire);
    }

    // Calibration support (optional - for future enhancement)
    void set_hard_iron_offset(const Eigen::Vector3f& offset) {
        hard_iron_offset_ = offset;
    }

    void set_soft_iron_matrix(const Eigen::Matrix3f& matrix) {
        soft_iron_matrix_ = matrix;
    }

private:
    // Sensor callback
    static int sensor_callback(int fd, int events, void* data);

    // Process sensor events
    void process_events();

    // Apply calibration
    Eigen::Vector3f calibrate(const Eigen::Vector3f& raw) const;

    // Sensor manager and queue
    ASensorManager* sensor_manager_;
    ASensorEventQueue* event_queue_;
    const ASensor* mag_sensor_;
    ALooper* looper_;

    // Lock-free queue
    LockFreeQueue<MagSample> mag_queue_;

    // State tracking
    std::atomic<bool> running_;
    std::atomic<uint64_t> overflow_count_;

    // Calibration parameters
    Eigen::Vector3f hard_iron_offset_;
    Eigen::Matrix3f soft_iron_matrix_;
};

}  // namespace fusion

#else
// Stub for non-Android platforms
namespace fusion {
class AndroidMag {
public:
    explicit AndroidMag(size_t = 64) {}
    bool start(float = 50.0f) { return false; }
    void stop() {}
    bool is_running() const { return false; }
    bool pop(MagSample&) { return false; }
    size_t queue_size() const { return 0; }
    size_t queue_capacity() const { return 0; }
    uint64_t get_overflow_count() const { return 0; }
    void set_hard_iron_offset(const Eigen::Vector3f&) {}
    void set_soft_iron_matrix(const Eigen::Matrix3f&) {}
};
}  // namespace fusion
#endif  // __ANDROID__
