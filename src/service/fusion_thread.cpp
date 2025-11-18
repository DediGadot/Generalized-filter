// Fusion Thread Implementation
//
// Real-time sensor fusion loop with adaptive rate and health monitoring

#include "fusion_thread.hpp"
#include "utils/logger.hpp"

#include <ctime>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <unistd.h>
#include <sched.h>

namespace fusion {

// === Constructor/Destructor ===

FusionThread::FusionThread(const FusionConfig& config)
    : thread_(0),
      running_(false),
      thread_created_(false),
      config_(config),
      last_cycle_time_ns_(0),
      cycle_start_time_ns_(0) {

    // Initialize filter
    ekf_state_.initialize(Vector3d::Zero(), Vector3d::Zero(),
                          Quaterniond::Identity(), 0);
    ekf_state_.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

    // Initialize magnetometer update (default location: San Francisco)
    // In production, this would be set from Android LocationManager
    mag_update_.set_location(37.7749, -122.4194, 100.0);

    // Initialize state
    current_state_.timestamp_ns = get_timestamp_ns();
    current_state_.sequence = 0;

    LOG_INFO("FusionThread created (base_rate: %.1f Hz, adaptive: %s)",
             config_.base_rate_hz,
             config_.adaptive_rate_enabled ? "enabled" : "disabled");
}

FusionThread::~FusionThread() {
    if (is_running()) {
        LOG_WARN("FusionThread destroyed while running, stopping...");
        stop();
    }
}

// === Thread Management ===

bool FusionThread::start() {
    if (running_.load(std::memory_order_acquire)) {
        LOG_WARN("FusionThread already running");
        return false;
    }

    running_.store(true, std::memory_order_release);

    // Create thread
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Set real-time priority (if permission available)
    if (config_.thread_priority > 0) {
        struct sched_param param;
        param.sched_priority = config_.thread_priority;

        int ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (ret == 0) {
            ret = pthread_attr_setschedparam(&attr, &param);
            if (ret == 0) {
                LOG_INFO("Real-time priority enabled (SCHED_FIFO, priority %d)",
                         config_.thread_priority);
            } else {
                LOG_WARN("Failed to set real-time parameters: %d", ret);
            }
        } else {
            LOG_WARN("Failed to set SCHED_FIFO policy: %d (normal priority used)", ret);
        }
    }

    int ret = pthread_create(&thread_, &attr, thread_entry, this);
    pthread_attr_destroy(&attr);

    if (ret != 0) {
        LOG_ERROR("Failed to create fusion thread: %d", ret);
        running_.store(false, std::memory_order_release);
        return false;
    }

    thread_created_ = true;
    LOG_INFO("FusionThread started");
    return true;
}

void FusionThread::stop() {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }

    LOG_INFO("Stopping FusionThread...");
    running_.store(false, std::memory_order_release);

    if (thread_created_) {
        pthread_join(thread_, nullptr);
        thread_created_ = false;
    }

    LOG_INFO("FusionThread stopped (total cycles: %lu)", stats_.cycle_count);
}

void* FusionThread::thread_entry(void* arg) {
    auto* thread = static_cast<FusionThread*>(arg);

    // Set CPU affinity if configured
    if (thread->config_.cpu_affinity >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(thread->config_.cpu_affinity, &cpuset);

        int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
        if (ret == 0) {
            LOG_INFO("CPU affinity set to core %d", thread->config_.cpu_affinity);
        } else {
            LOG_WARN("Failed to set CPU affinity: %d", ret);
        }
    }

    thread->run();
    return nullptr;
}

// === Main Fusion Loop ===

void FusionThread::run() {
    LOG_INFO("Fusion loop starting");

    last_cycle_time_ns_ = get_timestamp_ns();

    while (running_.load(std::memory_order_acquire)) {
        cycle_start_time_ns_ = get_timestamp_ns();

        // Execute fusion cycle
        fusion_cycle();

        // Update timing statistics
        int64_t cycle_end_ns = get_timestamp_ns();
        uint32_t cycle_time_us = static_cast<uint32_t>((cycle_end_ns - cycle_start_time_ns_) / 1000);

        stats_.cycle_count++;
        stats_.total_time_us += cycle_time_us;
        stats_.avg_cycle_time_us = static_cast<uint32_t>(stats_.total_time_us / stats_.cycle_count);

        if (cycle_time_us > stats_.max_cycle_time_us) {
            stats_.max_cycle_time_us = cycle_time_us;
        }
        if (cycle_time_us < stats_.min_cycle_time_us) {
            stats_.min_cycle_time_us = cycle_time_us;
        }

        // Compute adaptive rate
        double target_rate_hz = config_.adaptive_rate_enabled ?
                                compute_adaptive_rate() :
                                config_.base_rate_hz;

        stats_.current_rate_hz = target_rate_hz;

        // Sleep until next cycle
        sleep_until_next_cycle(cycle_start_time_ns_, target_rate_hz);

        last_cycle_time_ns_ = cycle_start_time_ns_;
    }

    LOG_INFO("Fusion loop exiting");
}

void FusionThread::fusion_cycle() {
    // Step 1: Preintegrate all pending IMU samples
    ImuSample sample;
    int imu_count = 0;

    while (imu_queue_.pop(sample)) {
        imu_preint_.integrate(sample);
        imu_count++;

        // Update IMU history for motion detection
        recent_imu_samples_.push_back(sample);
        if (recent_imu_samples_.size() > MAX_IMU_HISTORY) {
            recent_imu_samples_.erase(recent_imu_samples_.begin());
        }
    }

    stats_.imu_samples_processed += imu_count;

    if (imu_count == 0) {
        // No IMU data - critical failure
        check_sensor_health();
        return;
    }

    // Step 2: EKF Prediction
    auto preint_result = imu_preint_.get_result();
    Vector3d gravity(0, 0, 9.81);
    ekf_state_.predict(preint_result, gravity);
    imu_preint_.reset();

    // Step 3: Process magnetometer updates
    MagSample mag_sample;
    int mag_count = 0;

    while (mag_queue_.pop(mag_sample)) {
        bool accepted = mag_update_.update(ekf_state_, mag_sample.mag_body, true);
        if (accepted) {
            ekf_state_.inject_error(ekf_state_.error_state());
            ekf_state_.reset_error();
            mag_count++;
        }
    }

    stats_.mag_updates_processed += mag_count;

    // Step 4: Publish state (thread-safe)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        current_state_.position = ekf_state_.position();
        current_state_.velocity = ekf_state_.velocity();
        current_state_.orientation = ekf_state_.attitude();
        current_state_.gyro_bias = ekf_state_.gyro_bias();
        current_state_.accel_bias = ekf_state_.accel_bias();

        // Extract std dev from covariance diagonal
        const auto& P = ekf_state_.covariance();
        current_state_.attitude_std = P.block<3,3>(0,0).diagonal().array().sqrt();
        current_state_.velocity_std = P.block<3,3>(3,3).diagonal().array().sqrt();
        current_state_.position_std = P.block<3,3>(6,6).diagonal().array().sqrt();

        current_state_.timestamp_ns = get_timestamp_ns();
        current_state_.sequence++;
    }

    // Step 5: Health monitoring
    check_filter_divergence();
    check_thermal_throttling();
}

// === Adaptive Rate Control ===

double FusionThread::compute_adaptive_rate() {
    bool stationary = is_stationary();
    stats_.is_stationary = stationary;

    if (stationary) {
        return config_.stationary_rate_hz;
    }

    // Moving: scale rate based on motion magnitude
    float motion_mag = compute_motion_magnitude();
    double rate = config_.moving_rate_hz +
                  (config_.aggressive_rate_hz - config_.moving_rate_hz) * motion_mag;

    return std::clamp(rate, config_.stationary_rate_hz, config_.aggressive_rate_hz);
}

bool FusionThread::is_stationary() const {
    if (recent_imu_samples_.size() < 20) {
        return true;  // Not enough data, assume stationary
    }

    // Compute variance of accel and gyro over recent samples
    Vector3f accel_mean = Vector3f::Zero();
    Vector3f gyro_mean = Vector3f::Zero();

    for (const auto& sample : recent_imu_samples_) {
        accel_mean += sample.accel;
        gyro_mean += sample.gyro;
    }

    accel_mean /= static_cast<float>(recent_imu_samples_.size());
    gyro_mean /= static_cast<float>(recent_imu_samples_.size());

    Vector3f accel_var = Vector3f::Zero();
    Vector3f gyro_var = Vector3f::Zero();

    for (const auto& sample : recent_imu_samples_) {
        Vector3f accel_diff = sample.accel - accel_mean;
        Vector3f gyro_diff = sample.gyro - gyro_mean;
        accel_var += accel_diff.cwiseProduct(accel_diff);
        gyro_var += gyro_diff.cwiseProduct(gyro_diff);
    }

    accel_var /= static_cast<float>(recent_imu_samples_.size());
    gyro_var /= static_cast<float>(recent_imu_samples_.size());

    float accel_std = accel_var.norm();
    float gyro_std = gyro_var.norm();

    return (accel_std < config_.accel_stationary_threshold &&
            gyro_std < config_.gyro_stationary_threshold);
}

float FusionThread::compute_motion_magnitude() const {
    if (recent_imu_samples_.size() < 10) {
        return 0.0f;
    }

    // Simple heuristic: norm of recent gyro samples
    float gyro_norm_sum = 0.0f;
    size_t count = std::min(recent_imu_samples_.size(), size_t(20));

    for (size_t i = recent_imu_samples_.size() - count; i < recent_imu_samples_.size(); i++) {
        gyro_norm_sum += recent_imu_samples_[i].gyro.norm();
    }

    float avg_gyro_norm = gyro_norm_sum / static_cast<float>(count);

    // Scale to [0, 1] range (empirical scaling)
    return std::clamp(avg_gyro_norm * config_.motion_scale_factor, 0.0f, 1.0f);
}

// === Health Monitoring ===

void FusionThread::check_thermal_throttling() {
    if (!config_.thermal_monitoring_enabled) {
        return;
    }

    int temp_millic = read_cpu_temperature_millic();
    if (temp_millic < 0) {
        return;  // Temperature unavailable
    }

    float temp_c = temp_millic / 1000.0f;
    stats_.cpu_temperature_c = temp_c;

    if (temp_c > config_.thermal_critical_temp_c) {
        // Critical: reduce to minimum rate
        config_.base_rate_hz = config_.stationary_rate_hz;
        stats_.thermal_throttle_count++;
        LOG_WARN("Thermal critical (%.1f°C), reducing to %.0f Hz",
                 temp_c, config_.base_rate_hz);
    } else if (temp_c > config_.thermal_throttle_temp_c) {
        // Moderate: reduce by 50%
        config_.base_rate_hz = std::max(config_.stationary_rate_hz,
                                        config_.base_rate_hz * 0.5);
        LOG_INFO("Thermal warning (%.1f°C), reducing rate", temp_c);
    }
}

void FusionThread::check_filter_divergence() {
    const auto& P = ekf_state_.covariance();

    // Check position/velocity/attitude uncertainty
    Vector3d pos_std = P.block<3,3>(6,6).diagonal().array().sqrt();
    Vector3d vel_std = P.block<3,3>(3,3).diagonal().array().sqrt();
    Vector3d att_std = P.block<3,3>(0,0).diagonal().array().sqrt();

    stats_.position_std_norm = pos_std.norm();
    stats_.velocity_std_norm = vel_std.norm();
    stats_.attitude_std_norm = att_std.norm();

    // Divergence thresholds (conservative)
    const double POS_DIVERGE_THRESHOLD = 100.0;  // 100m
    const double VEL_DIVERGE_THRESHOLD = 10.0;   // 10 m/s
    const double ATT_DIVERGE_THRESHOLD = 0.5;    // ~30°

    if (pos_std.norm() > POS_DIVERGE_THRESHOLD ||
        vel_std.norm() > VEL_DIVERGE_THRESHOLD ||
        att_std.norm() > ATT_DIVERGE_THRESHOLD) {

        stats_.divergence_count++;
        LOG_ERROR("Filter divergence detected (pos: %.1f, vel: %.1f, att: %.1f rad)",
                  pos_std.norm(), vel_std.norm(), att_std.norm());

        // Re-initialize filter
        ekf_state_.initialize(Vector3d::Zero(), Vector3d::Zero(),
                              Quaterniond::Identity(), get_timestamp_ns());
        ekf_state_.set_initial_covariance(0.1, 0.5, 1.0, 0.001, 0.01);

        LOG_INFO("Filter reinitialized");
    }
}

void FusionThread::check_sensor_health() {
    // Check for IMU dropout (critical)
    if (imu_queue_.empty()) {
        LOG_WARN("IMU queue empty - sensor dropout?");
    }

    // Check for queue overflows
    if (stats_.imu_queue_overflows > 0) {
        LOG_WARN("IMU queue overflows: %u", stats_.imu_queue_overflows);
    }

    if (stats_.mag_queue_overflows > 0) {
        LOG_WARN("Mag queue overflows: %u", stats_.mag_queue_overflows);
    }
}

// === State Access ===

FusedState FusionThread::get_current_state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

Pose FusionThread::get_current_pose() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_.to_pose();
}

ThreadStats FusionThread::get_stats() const {
    // stats_ is mutable and mostly atomic/protected
    return stats_;
}

// === Timing Utilities ===

int64_t FusionThread::get_timestamp_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

void FusionThread::sleep_until_next_cycle(int64_t cycle_start_ns, double target_rate_hz) {
    int64_t period_ns = static_cast<int64_t>(1e9 / target_rate_hz);
    int64_t target_wake_ns = cycle_start_ns + period_ns;
    int64_t now_ns = get_timestamp_ns();

    if (now_ns < target_wake_ns) {
        int64_t sleep_ns = target_wake_ns - now_ns;

        struct timespec sleep_time;
        sleep_time.tv_sec = sleep_ns / 1000000000LL;
        sleep_time.tv_nsec = sleep_ns % 1000000000LL;

        nanosleep(&sleep_time, nullptr);
    } else {
        // Overrun - log warning if significant
        int64_t overrun_us = (now_ns - target_wake_ns) / 1000;
        if (overrun_us > 1000) {  // >1ms overrun
            LOG_WARN("Fusion cycle overrun: %ld µs (target rate: %.1f Hz)",
                     overrun_us, target_rate_hz);
        }
    }
}

int FusionThread::read_cpu_temperature_millic() {
    // Try multiple thermal zones (Android devices vary)
    for (int zone = 0; zone < 10; zone++) {
        std::ostringstream path;
        path << "/sys/class/thermal/thermal_zone" << zone << "/temp";

        std::ifstream temp_file(path.str());
        if (temp_file.is_open()) {
            int temp_millic;
            temp_file >> temp_millic;
            temp_file.close();

            if (temp_millic > 0 && temp_millic < 150000) {  // Sanity check (0-150°C)
                return temp_millic;
            }
        }
    }

    return -1;  // Temperature unavailable
}

}  // namespace fusion
