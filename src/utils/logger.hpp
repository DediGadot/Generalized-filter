/**
 * @file logger.hpp
 * @brief Unified logging infrastructure for desktop and Android
 *
 * Purpose: Provides consistent logging interface that works on both
 * desktop (stdout/stderr) and Android (__android_log_print).
 *
 * References:
 * - Android NDK logging: https://developer.android.com/ndk/reference/group/logging
 * - DESIGN.md: Android Service Integration section
 *
 * Sample Input:
 *   LOG_INFO("Filter rate: %d Hz", fusion_rate);
 *
 * Expected Output:
 *   Desktop: "[INFO] Filter rate: 100 Hz"
 *   Android: logcat shows "I/FusionFilter: Filter rate: 100 Hz"
 */

#ifndef FUSION_UTILS_LOGGER_HPP
#define FUSION_UTILS_LOGGER_HPP

#ifdef ANDROID
#include <android/log.h>
#define LOG_TAG "FusionFilter"
#define LOG_INFO(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOG_WARN(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
#define LOG_ERROR(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOG_DEBUG(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#else
#include <cstdio>
#define LOG_INFO(...) do { std::printf("[INFO] " __VA_ARGS__); std::printf("\n"); } while(0)
#define LOG_WARN(...) do { std::fprintf(stderr, "[WARN] " __VA_ARGS__); std::fprintf(stderr, "\n"); } while(0)
#define LOG_ERROR(...) do { std::fprintf(stderr, "[ERROR] " __VA_ARGS__); std::fprintf(stderr, "\n"); } while(0)
#define LOG_DEBUG(...) do { std::printf("[DEBUG] " __VA_ARGS__); std::printf("\n"); } while(0)
#endif

namespace fusion {

/**
 * @brief Filter performance metrics for logging
 */
struct FilterMetrics {
    double prediction_time_us;      ///< EKF prediction time [µs]
    double update_time_us;          ///< EKF update time [µs]
    int imu_samples_processed;      ///< Number of IMU samples
    int gnss_updates;               ///< Number of GNSS updates
    int mag_updates;                ///< Number of MAG updates

    FilterMetrics()
        : prediction_time_us(0.0), update_time_us(0.0),
          imu_samples_processed(0), gnss_updates(0), mag_updates(0) {}
};

/**
 * @brief Log filter performance metrics
 */
inline void log_metrics(const FilterMetrics& m) {
    LOG_INFO("Metrics: Pred=%.1f µs, Update=%.1f µs, IMU=%d, GNSS=%d, MAG=%d",
             m.prediction_time_us, m.update_time_us,
             m.imu_samples_processed, m.gnss_updates, m.mag_updates);
}

} // namespace fusion

#endif // FUSION_UTILS_LOGGER_HPP
