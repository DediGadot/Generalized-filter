// JNI Bridge for GNSS Raw Measurements
//
// Purpose: Receive GNSS pseudorange/Doppler data from Android LocationManager
// Documentation: https://developer.android.com/reference/android/location/GnssMeasurement
//
// Architecture:
//   Java (GnssService) → JNI → C++ (this file) → Lock-free queue
//
// Key Features:
// - Raw GNSS measurements (pseudoranges, Doppler)
// - Satellite positions from ephemeris
// - Clock bias/drift estimates
// - Lock-free queue for thread-safe data transfer
//
// Sample Usage (C++):
//   JniGnss gnss;
//   // Java side registers callbacks...
//   GnssMeas meas;
//   if (gnss.pop(meas)) {
//       // Process GNSS measurement
//   }
//
// Sample Usage (Java):
//   GnssService service = new GnssService(context);
//   service.start();
//
// Expected Behavior:
//   - GNSS measurements at 1-10 Hz
//   - Multiple satellites per epoch
//   - Raw pseudoranges + Doppler shifts
//
// Platform: Android API 24+ (requires ACCESS_FINE_LOCATION permission)

#pragma once

#ifdef __ANDROID__

#include <jni.h>
#include <atomic>
#include <memory>

#include "core/lockfree_queue.hpp"
#include "core/sensor_types.hpp"

namespace fusion {

// GNSS queue uses the main measurement struct; keep alias for brevity.
using GnssMeas = GnssMeasurement;

class JniGnss {
public:
    // Constructor
    // queue_size: Power-of-2 capacity (default 16 - GNSS is slow)
    explicit JniGnss(size_t queue_size = 16);

    ~JniGnss();

    // Non-copyable, non-movable
    JniGnss(const JniGnss&) = delete;
    JniGnss& operator=(const JniGnss&) = delete;
    JniGnss(JniGnss&&) = delete;
    JniGnss& operator=(JniGnss&&) = delete;

    // Start GNSS measurement callbacks (call from Java side)
    bool start();

    // Stop callbacks
    void stop();

    // Check if active
    bool is_running() const { return running_.load(std::memory_order_acquire); }

    // Pop GNSS measurement from queue (non-blocking)
    // Returns: true if measurement retrieved, false if queue empty
    bool pop(GnssMeas& meas);

    // Get queue statistics
    size_t queue_size() const { return gnss_queue_.size(); }
    size_t queue_capacity() const { return gnss_queue_.capacity(); }

    // Get overflow counter
    uint64_t get_overflow_count() const {
        return overflow_count_.load(std::memory_order_acquire);
    }

    // ===== JNI CALLBACK INTERFACE =====
    // These methods are called from Java via JNI
    // Do NOT call directly from C++ code

    // Called when GNSS measurement arrives
    // timestamp_ns: System timestamp
    // num_sats: Number of satellites
    // prns: Satellite PRN numbers (array)
    // pseudoranges: Pseudorange measurements in meters (array)
    // dopplers: Doppler shift in m/s (array)
    // cn0s: Carrier-to-noise ratio in dB-Hz (array)
    void onGnssMeasurement(
        int64_t timestamp_ns,
        int num_sats,
        const int* prns,
        const double* pseudoranges,
        const float* dopplers,
        const float* cn0s);

    // Get singleton instance (for JNI callbacks)
    static JniGnss& instance();

private:
    // Lock-free queue
    LockFreeQueue<GnssMeas> gnss_queue_;

    // State tracking
    std::atomic<bool> running_;
    std::atomic<uint64_t> overflow_count_;
};

}  // namespace fusion

// ===== JNI FUNCTIONS =====
// These are exported for Java to call

extern "C" {

// Initialize GNSS bridge
JNIEXPORT void JNICALL
Java_com_fusion_GnssService_nativeInit(JNIEnv* env, jobject obj);

// Start GNSS measurements
JNIEXPORT jboolean JNICALL
Java_com_fusion_GnssService_nativeStart(JNIEnv* env, jobject obj);

// Stop GNSS measurements
JNIEXPORT void JNICALL
Java_com_fusion_GnssService_nativeStop(JNIEnv* env, jobject obj);

// Callback when GNSS measurement arrives
JNIEXPORT void JNICALL
Java_com_fusion_GnssService_nativeOnMeasurement(
    JNIEnv* env,
    jobject obj,
    jlong timestamp_ns,
    jint num_sats,
    jintArray prns,
    jdoubleArray pseudoranges,
    jfloatArray dopplers,
    jfloatArray cn0s);

}  // extern "C"

#else
// Stub for non-Android platforms
namespace fusion {
class JniGnss {
public:
    explicit JniGnss(size_t = 16) {}
    bool start() { return false; }
    void stop() {}
    bool is_running() const { return false; }
    bool pop(GnssMeas&) { return false; }
    size_t queue_size() const { return 0; }
    size_t queue_capacity() const { return 0; }
    uint64_t get_overflow_count() const { return 0; }

    void onGnssMeasurement(int64_t, int, const int*, const double*,
                          const float*, const float*) {}

    static JniGnss& instance() {
        static JniGnss inst;
        return inst;
    }
};
}  // namespace fusion
#endif  // __ANDROID__
