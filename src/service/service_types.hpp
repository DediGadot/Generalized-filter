// Service Data Types for Android Integration
//
// Purpose: Data structures for Android service layer and JNI bridge
// Reference: Phase 6 - Android Service Integration
//
// Key Features:
// - Fused state output (position, velocity, attitude, covariance)
// - Thread statistics (timing, health, diagnostics)
// - Pose representation (for apps querying position/orientation)
// - Lock-free atomic state for low-latency access
//
// Sample Usage:
//   FusedState state = fusion_thread.get_current_state();
//   Pose pose = state.to_pose();
//   ThreadStats stats = fusion_thread.get_stats();
//
// Expected Output:
//   - Low-latency state access (<1µs via atomic)
//   - Thread-safe concurrent reads
//   - Minimal JNI marshalling overhead

#pragma once

#include "core/quaternion.hpp"
#include "core/types.hpp"

#include <Eigen/Dense>
#include <cstdint>
#include <atomic>

namespace fusion {

/**
 * @brief 6DOF Pose (position + orientation)
 *
 * Lightweight representation for apps that only need pose (no velocity/covariance).
 * Suitable for direct JNI marshalling.
 */
struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vector3d position;     ///< Position in navigation frame [m]
    Quaterniond orientation; ///< Orientation (body → navigation)
    int64_t timestamp_ns;  ///< State timestamp [nanoseconds]

    Pose()
        : position(Vector3d::Zero()),
          orientation(Quaterniond::Identity()),
          timestamp_ns(0) {}
};

/**
 * @brief Complete fused state with covariance
 *
 * Full filter output including velocity and uncertainty.
 * Used for apps that need velocity or covariance information.
 */
struct FusedState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // State
    Vector3d position;       ///< Position in navigation frame [m]
    Vector3d velocity;       ///< Velocity in navigation frame [m/s]
    Quaterniond orientation; ///< Orientation (body → navigation)

    // Biases (for advanced users)
    Vector3d gyro_bias;      ///< Gyroscope bias [rad/s]
    Vector3d accel_bias;     ///< Accelerometer bias [m/s²]

    // Covariance (position, velocity, attitude)
    // Full 15×15 error covariance can be queried separately if needed
    Vector3d position_std;   ///< Position uncertainty (1-sigma) [m]
    Vector3d velocity_std;   ///< Velocity uncertainty (1-sigma) [m/s]
    Vector3d attitude_std;   ///< Attitude uncertainty (1-sigma) [rad]

    // Metadata
    int64_t timestamp_ns;    ///< State timestamp [nanoseconds]
    uint64_t sequence;       ///< Monotonic sequence number

    FusedState()
        : position(Vector3d::Zero()),
          velocity(Vector3d::Zero()),
          orientation(Quaterniond::Identity()),
          gyro_bias(Vector3d::Zero()),
          accel_bias(Vector3d::Zero()),
          position_std(Vector3d::Ones()),
          velocity_std(Vector3d::Ones()),
          attitude_std(Vector3d::Ones()),
          timestamp_ns(0),
          sequence(0) {}

    /**
     * @brief Convert to simple Pose (drops velocity, covariance)
     */
    Pose to_pose() const {
        Pose pose;
        pose.position = position;
        pose.orientation = orientation;
        pose.timestamp_ns = timestamp_ns;
        return pose;
    }
};

/**
 * @brief Thread statistics and diagnostics
 *
 * Performance metrics and health indicators for monitoring.
 */
struct ThreadStats {
    // Timing statistics
    uint64_t cycle_count;         ///< Total fusion cycles executed
    uint64_t total_time_us;       ///< Total CPU time consumed [µs]
    uint32_t avg_cycle_time_us;   ///< Average cycle time [µs]
    uint32_t max_cycle_time_us;   ///< Maximum cycle time [µs]
    uint32_t min_cycle_time_us;   ///< Minimum cycle time [µs]

    // Rate statistics
    double current_rate_hz;       ///< Current fusion rate [Hz]
    double avg_rate_hz;           ///< Average fusion rate [Hz]

    // Health indicators
    bool is_stationary;           ///< Motion detection state
    float cpu_temperature_c;      ///< CPU temperature [°C]
    uint32_t thermal_throttle_count; ///< Number of thermal throttle events

    // Sensor health
    uint64_t imu_samples_processed;   ///< Total IMU samples processed
    uint64_t mag_updates_processed;   ///< Total magnetometer updates
    uint64_t gnss_updates_processed;  ///< Total GNSS updates
    uint32_t imu_queue_overflows;     ///< IMU queue overflow count
    uint32_t mag_queue_overflows;     ///< Mag queue overflow count

    // Filter health
    double position_std_norm;     ///< Position uncertainty magnitude [m]
    double velocity_std_norm;     ///< Velocity uncertainty magnitude [m/s]
    double attitude_std_norm;     ///< Attitude uncertainty magnitude [rad]
    uint32_t divergence_count;    ///< Filter divergence detection count

    ThreadStats()
        : cycle_count(0),
          total_time_us(0),
          avg_cycle_time_us(0),
          max_cycle_time_us(0),
          min_cycle_time_us(UINT32_MAX),
          current_rate_hz(0.0),
          avg_rate_hz(0.0),
          is_stationary(true),
          cpu_temperature_c(0.0f),
          thermal_throttle_count(0),
          imu_samples_processed(0),
          mag_updates_processed(0),
          gnss_updates_processed(0),
          imu_queue_overflows(0),
          mag_queue_overflows(0),
          position_std_norm(0.0),
          velocity_std_norm(0.0),
          attitude_std_norm(0.0),
          divergence_count(0) {}
};

/**
 * @brief Fusion thread configuration
 *
 * Runtime configuration for fusion thread behavior.
 */
struct FusionConfig {
    // Rate control
    double base_rate_hz;          ///< Base fusion rate [Hz] (default: 100)
    bool adaptive_rate_enabled;   ///< Enable adaptive rate (default: true)
    double stationary_rate_hz;    ///< Rate when stationary (default: 50)
    double moving_rate_hz;        ///< Rate when moving (default: 100)
    double aggressive_rate_hz;    ///< Rate during fast motion (default: 200)

    // Motion detection thresholds
    float accel_stationary_threshold;  ///< Accel variance threshold [m/s²]
    float gyro_stationary_threshold;   ///< Gyro variance threshold [rad/s]
    float motion_scale_factor;         ///< Scale factor for adaptive rate

    // Thermal management
    bool thermal_monitoring_enabled;   ///< Enable thermal monitoring
    float thermal_throttle_temp_c;     ///< Temperature to throttle [°C]
    float thermal_critical_temp_c;     ///< Critical temperature [°C]

    // Thread configuration
    int thread_priority;          ///< Thread priority (0-99, 0=normal)
    int cpu_affinity;             ///< CPU core affinity (-1=none, 0-7=core)

    // Queue sizes
    uint32_t imu_queue_size;      ///< IMU queue capacity
    uint32_t mag_queue_size;      ///< Magnetometer queue capacity
    uint32_t gnss_queue_size;     ///< GNSS queue capacity

    FusionConfig()
        : base_rate_hz(100.0),
          adaptive_rate_enabled(true),
          stationary_rate_hz(50.0),
          moving_rate_hz(100.0),
          aggressive_rate_hz(200.0),
          accel_stationary_threshold(0.05f),
          gyro_stationary_threshold(0.01f),
          motion_scale_factor(150.0f),
          thermal_monitoring_enabled(true),
          thermal_throttle_temp_c(70.0f),
          thermal_critical_temp_c(80.0f),
          thread_priority(0),  // Normal priority (SCHED_FIFO requires root)
          cpu_affinity(-1),    // No affinity
          imu_queue_size(512),
          mag_queue_size(64),
          gnss_queue_size(16) {}
};

}  // namespace fusion
