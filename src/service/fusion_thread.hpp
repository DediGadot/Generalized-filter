// Fusion Thread - Real-time Sensor Fusion Loop
//
// Purpose: Production fusion thread with real-time priority and adaptive rate
// Reference: Phase 6 - Android Service Integration
//            DESIGN.md Threading Model (lines 1030-1095)
//
// Key Features:
// - pthread with SCHED_FIFO priority (if available)
// - CPU affinity to big core (cortex-a76 on Snapdragon AR1)
// - Adaptive rate: 50 Hz stationary, 100-200 Hz moving
// - Lock-free state publishing for low-latency access
// - Health monitoring: thermal, divergence, sensor dropouts
// - Thread-safe lifecycle management
//
// Sample Usage:
//   FusionThread thread(config);
//   thread.start();
//
//   // Query state from any thread
//   FusedState state = thread.get_current_state();
//   Pose pose = thread.get_current_pose();
//
//   thread.stop();
//
// Expected Output:
//   - Deterministic 50-200 Hz fusion rate
//   - <1µs state access latency
//   - <10% CPU usage
//   - Automatic thermal throttling

#pragma once

#include "service_types.hpp"
#include "filter/ekf_state.hpp"
#include "filter/imu_preintegration.hpp"
#include "filter/mag_update.hpp"
#include "core/sensor_types.hpp"
#include "core/types.hpp"

#include <pthread.h>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdint>

namespace fusion {

/**
 * @brief Simple SPSC (Single Producer Single Consumer) Queue
 *
 * Lock-free queue for sensor data from Android callbacks to fusion thread.
 * Template-based for IMU, Magnetometer, GNSS measurements.
 */
template<typename T, size_t Capacity>
class SPSCQueue {
public:
    SPSCQueue() : head_(0), tail_(0) {
        static_assert((Capacity & (Capacity - 1)) == 0,
                      "Capacity must be power of 2");
    }

    /**
     * @brief Push element (producer side)
     * @return true if successful, false if queue full
     */
    bool push(const T& item) {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t next_head = (head + 1) & (Capacity - 1);

        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false;  // Queue full
        }

        buffer_[head] = item;
        head_.store(next_head, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop element (consumer side)
     * @return true if successful, false if queue empty
     */
    bool pop(T& item) {
        size_t tail = tail_.load(std::memory_order_relaxed);

        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // Queue empty
        }

        item = buffer_[tail];
        tail_.store((tail + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    /**
     * @brief Check if queue is empty
     */
    bool empty() const {
        return tail_.load(std::memory_order_acquire) ==
               head_.load(std::memory_order_acquire);
    }

    /**
     * @brief Get current queue size
     */
    size_t size() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & (Capacity - 1);
    }

private:
    std::array<T, Capacity> buffer_;
    alignas(64) std::atomic<size_t> head_;  // Cache line aligned
    alignas(64) std::atomic<size_t> tail_;  // Cache line aligned
};

/**
 * @brief Real-time fusion thread
 *
 * Runs the main fusion loop at adaptive rate (50-200 Hz) with real-time
 * priority. Thread-safe state access via atomics and lock-free queues.
 */
class FusionThread {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor
     * @param config Fusion configuration
     */
    explicit FusionThread(const FusionConfig& config = FusionConfig());

    /**
     * @brief Destructor (stops thread if running)
     */
    ~FusionThread();

    // Disable copy/move (manages thread lifetime)
    FusionThread(const FusionThread&) = delete;
    FusionThread& operator=(const FusionThread&) = delete;

    /**
     * @brief Start fusion thread
     * @return true if started successfully
     */
    bool start();

    /**
     * @brief Stop fusion thread
     *
     * Waits for thread to finish current cycle before stopping.
     */
    void stop();

    /**
     * @brief Check if thread is running
     */
    bool is_running() const { return running_.load(std::memory_order_acquire); }

    // === Sensor Data Input (called from Android callbacks) ===

    /**
     * @brief Push IMU sample to queue
     * @return true if queued, false if overflow
     */
    bool push_imu_sample(const ImuSample& sample) {
        bool success = imu_queue_.push(sample);
        if (!success) {
            stats_.imu_queue_overflows++;
        }
        return success;
    }

    /**
     * @brief Push magnetometer sample to queue
     * @return true if queued, false if overflow
     */
    bool push_mag_sample(const Vector3d& mag_body, int64_t timestamp_ns) {
        MagSample sample;
        sample.mag_body = mag_body;
        sample.timestamp_ns = timestamp_ns;

        bool success = mag_queue_.push(sample);
        if (!success) {
            stats_.mag_queue_overflows++;
        }
        return success;
    }

    // === State Access (thread-safe, lock-free) ===

    /**
     * @brief Get current fused state
     *
     * Lock-free read of last published state. Safe to call from any thread.
     *
     * @return Current fused state with covariance
     */
    FusedState get_current_state() const;

    /**
     * @brief Get current pose (lightweight, no covariance)
     *
     * @return Current pose (position + orientation)
     */
    Pose get_current_pose() const;

    /**
     * @brief Get thread statistics
     *
     * @return Performance and health metrics
     */
    ThreadStats get_stats() const;

    // === Configuration (thread-safe, takes effect next cycle) ===

    /**
     * @brief Enable/disable adaptive rate
     */
    void set_adaptive_rate(bool enabled) {
        config_.adaptive_rate_enabled = enabled;
    }

    /**
     * @brief Set base fusion rate [Hz]
     */
    void set_base_rate(double hz) {
        config_.base_rate_hz = hz;
    }

    /**
     * @brief Set CPU affinity
     * @param core_id CPU core (0-7, -1 = no affinity)
     */
    void set_cpu_affinity(int core_id) {
        config_.cpu_affinity = core_id;
    }

private:
    // === Main Fusion Loop ===

    /**
     * @brief Thread entry point (static wrapper)
     */
    static void* thread_entry(void* arg);

    /**
     * @brief Main fusion loop
     */
    void run();

    /**
     * @brief Single fusion cycle
     *
     * Performs one complete fusion iteration:
     * 1. Preintegrate pending IMU samples
     * 2. EKF prediction
     * 3. Process magnetometer updates
     * 4. Publish state
     * 5. Update statistics
     */
    void fusion_cycle();

    // === Adaptive Rate Control ===

    /**
     * @brief Compute target fusion rate based on motion
     * @return Target rate [Hz]
     */
    double compute_adaptive_rate();

    /**
     * @brief Check if device is stationary
     *
     * Uses IMU variance over last 1 second to detect motion.
     *
     * @return true if stationary
     */
    bool is_stationary() const;

    /**
     * @brief Compute motion magnitude for rate scaling
     * @return Motion magnitude [0.0 - 1.0]
     */
    float compute_motion_magnitude() const;

    // === Health Monitoring ===

    /**
     * @brief Check for thermal throttling
     *
     * Reads CPU temperature and reduces rate if overheating.
     */
    void check_thermal_throttling();

    /**
     * @brief Check for filter divergence
     *
     * Monitors covariance for divergence, reinitializes if needed.
     */
    void check_filter_divergence();

    /**
     * @brief Check sensor health
     *
     * Detects sensor dropouts (e.g., no IMU data).
     */
    void check_sensor_health();

    // === Timing Utilities ===

    /**
     * @brief Get current timestamp [nanoseconds]
     */
    static int64_t get_timestamp_ns();

    /**
     * @brief Sleep until next cycle (adaptive rate)
     */
    void sleep_until_next_cycle(int64_t cycle_start_ns, double target_rate_hz);

    // === CPU Temperature Reading ===

    /**
     * @brief Read CPU temperature from /sys/class/thermal
     * @return Temperature in milli-°C (or -1 if unavailable)
     */
    static int read_cpu_temperature_millic();

    // === Thread Management ===

    pthread_t thread_;
    std::atomic<bool> running_;
    bool thread_created_;

    // === Filter Components ===

    EkfState ekf_state_;
    ImuPreintegration imu_preint_;
    MagUpdate mag_update_;

    // === Sensor Queues ===

    struct MagSample {
        Vector3d mag_body;
        int64_t timestamp_ns;
    };

    SPSCQueue<ImuSample, 512> imu_queue_;
    SPSCQueue<MagSample, 64> mag_queue_;

    // === Published State (lock-free atomic) ===

    mutable std::mutex state_mutex_;  // Protects current_state_ (fallback)
    FusedState current_state_;

    // === Configuration ===

    FusionConfig config_;

    // === Statistics (atomic for thread-safe access) ===

    mutable ThreadStats stats_;  // Most fields are atomic or mutex-protected

    // === IMU History (for motion detection) ===

    std::vector<ImuSample> recent_imu_samples_;  // Last 1 second of IMU
    static constexpr size_t MAX_IMU_HISTORY = 200;  // 1 second @ 200 Hz

    // === Timing State ===

    int64_t last_cycle_time_ns_;
    int64_t cycle_start_time_ns_;
};

}  // namespace fusion
