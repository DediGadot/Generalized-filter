// JNI Bridge for Android Fused Location - Implementation

#include "jni_location.hpp"
#include <cmath>

namespace fusion {

// WGS84 ellipsoid constants
constexpr double WGS84_A = 6378137.0;           // Semi-major axis [m]
constexpr double WGS84_F = 1.0 / 298.257223563; // Flattening
constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F;  // Eccentricity squared

JniLocation::JniLocation(size_t queue_size)
    : location_queue_(queue_size),
      running_(false),
      overflow_count_(0),
      reference_set_(false),
      ref_lat_deg_(0.0),
      ref_lon_deg_(0.0),
      ref_alt_m_(0.0) {
}

bool JniLocation::start(LocationPriority priority, double interval_sec) {
    (void)priority;      // Will be used by Java side
    (void)interval_sec;  // Will be used by Java side

    running_.store(true, std::memory_order_release);
    return true;
}

void JniLocation::stop() {
    running_.store(false, std::memory_order_release);
}

bool JniLocation::pop(LocationMeasurement& meas) {
    return location_queue_.pop(meas);
}

bool JniLocation::push_location(
    int64_t timestamp_ns,
    double latitude_deg,
    double longitude_deg,
    double altitude_m,
    float horizontal_accuracy_m,
    float vertical_accuracy_m,
    float speed_mps,
    float speed_accuracy_mps,
    int provider_flags) {

    LocationMeasurement meas;
    meas.timestamp_ns = timestamp_ns;
    meas.latitude_deg = latitude_deg;
    meas.longitude_deg = longitude_deg;
    meas.altitude_m = altitude_m;
    meas.horizontal_accuracy_m = horizontal_accuracy_m;
    meas.vertical_accuracy_m = vertical_accuracy_m;
    meas.speed_mps = speed_mps;
    meas.speed_accuracy_mps = speed_accuracy_mps;
    meas.provider_flags = static_cast<uint8_t>(provider_flags);

    // Convert WGS84 to NED
    meas.position_ned = wgs84_to_ned(latitude_deg, longitude_deg, altitude_m);

    // Push to queue
    bool success = location_queue_.push(meas);
    if (!success) {
        overflow_count_.fetch_add(1, std::memory_order_relaxed);
    }

    return success;
}

Vector3d JniLocation::wgs84_to_ned(double lat_deg, double lon_deg, double alt_m) {
    // Set reference point on first location fix
    if (!reference_set_) {
        ref_lat_deg_ = lat_deg;
        ref_lon_deg_ = lon_deg;
        ref_alt_m_ = alt_m;
        reference_set_ = true;
        // First fix is at origin
        return Vector3d::Zero();
    }

    // Convert to radians
    const double lat_rad = lat_deg * M_PI / 180.0;
    const double lon_rad = lon_deg * M_PI / 180.0;
    const double ref_lat_rad = ref_lat_deg_ * M_PI / 180.0;
    const double ref_lon_rad = ref_lon_deg_ * M_PI / 180.0;

    // Radius of curvature in the prime vertical
    const double sin_lat = std::sin(ref_lat_rad);
    const double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    // Tangent plane approximation (valid for <10km radius)
    // For larger distances, use full ECEF conversion
    const double delta_lat = lat_rad - ref_lat_rad;
    const double delta_lon = lon_rad - ref_lon_rad;
    const double delta_alt = alt_m - ref_alt_m_;

    // NED coordinates
    const double north = delta_lat * (N + ref_alt_m_);
    const double east = delta_lon * (N + ref_alt_m_) * std::cos(ref_lat_rad);
    const double down = -delta_alt;  // NED uses down as positive

    return Vector3d(north, east, down);
}

JniLocation& JniLocation::instance() {
    static JniLocation inst;
    return inst;
}

}  // namespace fusion

// === JNI IMPLEMENTATIONS ===

#ifdef __ANDROID__

extern "C" {

JNIEXPORT void JNICALL
Java_com_fusion_LocationService_nativeInit(JNIEnv* env, jobject obj) {
    (void)env;
    (void)obj;
    // Instance already created as singleton
    fusion::JniLocation::instance();
}

JNIEXPORT jboolean JNICALL
Java_com_fusion_LocationService_nativeStart(
    JNIEnv* env, jobject obj, jint priority, jdouble interval_sec) {
    (void)env;
    (void)obj;

    auto prio = static_cast<fusion::LocationPriority>(priority);
    bool success = fusion::JniLocation::instance().start(prio, interval_sec);
    return success ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_fusion_LocationService_nativeStop(JNIEnv* env, jobject obj) {
    (void)env;
    (void)obj;
    fusion::JniLocation::instance().stop();
}

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
    jint provider_flags) {
    (void)env;
    (void)obj;

    fusion::JniLocation::instance().push_location(
        static_cast<int64_t>(timestamp_ns),
        latitude,
        longitude,
        altitude,
        horizontal_accuracy,
        vertical_accuracy,
        speed,
        speed_accuracy,
        static_cast<int>(provider_flags)
    );
}

}  // extern "C"

#endif  // __ANDROID__
