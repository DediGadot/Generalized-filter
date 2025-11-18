// JNI Bridge for Android Fused Location
//
// Purpose: Receive pre-computed location from Android FusedLocationProvider
// Documentation: https://developer.android.com/training/location/retrieve-current
//
// Architecture:
//   Java (FusedLocationProviderClient) → JNI → C++ → Lock-free queue
//
// Key Features:
// - Multi-source fusion (GPS + WiFi + cell + BLE)
// - Automatic indoor/outdoor switching
// - Battery-optimized location updates
// - Priority-based update rates (HIGH_ACCURACY, BALANCED_POWER, etc.)
//
// Sample Usage (C++):
//   JniLocation location;
//   location.start(LocationPriority::HIGH_ACCURACY, 1.0);  // 1 Hz
//
//   LocationMeasurement meas;
//   if (location.pop(meas)) {
//       // Process location measurement
//   }
//
// Expected Output:
//   - Location updates at 0.5-5 Hz
//   - Automatic WGS84→NED conversion
//   - Thread-safe queue interface

#pragma once

#include "core/lockfree_queue.hpp"
#include "core/sensor_types.hpp"
#include "core/types.hpp"

#include <atomic>
#include <memory>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fusion {

/**
 * @brief Location update priority (maps to Android Priority constants)
 */
enum class LocationPriority {
    HIGH_ACCURACY = 100,      ///< GPS-based, ~5-10m accuracy, 1-5 Hz
    BALANCED_POWER = 102,     ///< WiFi + GPS, ~20-100m, 0.5-1 Hz
    LOW_POWER = 104,          ///< Cell + WiFi, ~500m-1km, <0.5 Hz
    PASSIVE = 105             ///< Piggyback on other apps' requests
};

/**
 * @brief JNI bridge for Android Fused Location
 *
 * Receives location updates from Java and pushes to lock-free queue
 */
class JniLocation {
public:
    /**
     * @brief Constructor
     * @param queue_size Power-of-2 capacity (default 16 - location is slow)
     */
    explicit JniLocation(size_t queue_size = 16);

    ~JniLocation() = default;

    // Non-copyable, non-movable
    JniLocation(const JniLocation&) = delete;
    JniLocation& operator=(const JniLocation&) = delete;
    JniLocation(JniLocation&&) = delete;
    JniLocation& operator=(JniLocation&&) = delete;

    /**
     * @brief Start location updates
     *
     * @param priority Location priority (accuracy vs battery)
     * @param interval_sec Update interval in seconds (0.2 - 10.0)
     * @return true if started successfully
     */
    bool start(LocationPriority priority = LocationPriority::HIGH_ACCURACY,
               double interval_sec = 1.0);

    /**
     * @brief Stop location updates
     */
    void stop();

    /**
     * @brief Check if running
     */
    bool is_running() const { return running_.load(std::memory_order_acquire); }

    /**
     * @brief Pop location measurement from queue (non-blocking)
     * @return true if measurement retrieved, false if queue empty
     */
    bool pop(LocationMeasurement& meas);

    /**
     * @brief Push location measurement to queue (called from JNI callback)
     *
     * @param timestamp_ns System timestamp
     * @param latitude_deg Latitude in degrees
     * @param longitude_deg Longitude in degrees
     * @param altitude_m Altitude in meters above WGS84 ellipsoid
     * @param horizontal_accuracy_m Horizontal accuracy (1-sigma)
     * @param vertical_accuracy_m Vertical accuracy (1-sigma)
     * @param speed_mps Speed in m/s
     * @param speed_accuracy_mps Speed accuracy
     * @param provider_flags Bit flags (GPS=1, WiFi=2, Cell=4, BLE=8)
     * @return true if queued, false if overflow
     */
    bool push_location(
        int64_t timestamp_ns,
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        float horizontal_accuracy_m,
        float vertical_accuracy_m,
        float speed_mps,
        float speed_accuracy_mps,
        int provider_flags);

    // Queue statistics
    size_t queue_size() const { return location_queue_.size(); }
    size_t queue_capacity() const { return location_queue_.capacity(); }
    uint64_t get_overflow_count() const {
        return overflow_count_.load(std::memory_order_acquire);
    }

    /**
     * @brief Get singleton instance (for JNI callbacks)
     */
    static JniLocation& instance();

private:
    /**
     * @brief Convert WGS84 to local NED frame
     *
     * Uses simple tangent plane approximation (valid for <10km radius).
     * Reference point is set on first location fix.
     *
     * @param lat_deg Latitude in degrees
     * @param lon_deg Longitude in degrees
     * @param alt_m Altitude in meters
     * @return Position in NED frame [North, East, Down] in meters
     */
    Vector3d wgs84_to_ned(double lat_deg, double lon_deg, double alt_m);

    // Lock-free queue for location measurements
    LockFreeQueue<LocationMeasurement> location_queue_;

    // State tracking
    std::atomic<bool> running_;
    std::atomic<uint64_t> overflow_count_;

    // Reference point for NED frame conversion
    // Set on first location fix
    bool reference_set_;
    double ref_lat_deg_;
    double ref_lon_deg_;
    double ref_alt_m_;
};

}  // namespace fusion

// === JNI FUNCTIONS ===
// These are exported for Java to call

#ifdef __ANDROID__

#include <jni.h>

extern "C" {

/**
 * @brief Initialize location bridge
 */
JNIEXPORT void JNICALL
Java_com_fusion_LocationService_nativeInit(JNIEnv* env, jobject obj);

/**
 * @brief Start location updates
 */
JNIEXPORT jboolean JNICALL
Java_com_fusion_LocationService_nativeStart(
    JNIEnv* env, jobject obj, jint priority, jdouble interval_sec);

/**
 * @brief Stop location updates
 */
JNIEXPORT void JNICALL
Java_com_fusion_LocationService_nativeStop(JNIEnv* env, jobject obj);

/**
 * @brief Callback when location arrives
 */
JNIEXPORT void JNICALL
Java_com_fusion_LocationService_nativeOnLocation(
    JNIEnv* env,
    jobject obj,
    jlong timestamp_ns,
    jdouble latitude,
    jdouble longitude,
    jdouble altitude,
    jfloat horizontal_accuracy,
    jfloat vertical_accuracy,
    jfloat speed,
    jfloat speed_accuracy,
    jint provider_flags);

}  // extern "C"

#endif  // __ANDROID__
