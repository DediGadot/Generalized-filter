// JNI GNSS Implementation
#ifdef __ANDROID__

#include "sensors/jni_gnss.hpp"
#include "utils/logger.hpp"

#include <algorithm>
#include <cstring>

namespace fusion {

JniGnss::JniGnss(size_t queue_size)
    : gnss_queue_(queue_size),
      running_(false),
      overflow_count_(0) {
    LOG_INFO("JniGnss initialized with queue size {}", queue_size);
}

JniGnss::~JniGnss() {
    stop();
}

bool JniGnss::start() {
    if (running_.load(std::memory_order_acquire)) {
        LOG_WARN("GNSS already running");
        return false;
    }

    running_.store(true, std::memory_order_release);
    LOG_INFO("GNSS measurements started");
    return true;
}

void JniGnss::stop() {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }

    running_.store(false, std::memory_order_release);
    LOG_INFO("GNSS measurements stopped (overflow count: {})",
             overflow_count_.load());
}

bool JniGnss::pop(GnssMeas& meas) {
    return gnss_queue_.pop(meas);
}

void JniGnss::onGnssMeasurement(
    int64_t timestamp_ns,
    int num_sats,
    const int* prns,
    const double* pseudoranges,
    const float* dopplers,
    const float* cn0s) {

    if (!running_.load(std::memory_order_acquire)) {
        return;  // Ignore measurements when not active
    }

    // Clamp number of satellites
    const int max_sats = static_cast<int>(MAX_GNSS_SATS);
    const int actual_sats = std::min(num_sats, max_sats);

    if (num_sats > max_sats) {
        LOG_WARN("Received {} satellites, limiting to {}", num_sats, max_sats);
    }

    // Create GNSS measurement
    GnssMeas meas;
    meas.timestamp_ns = timestamp_ns;
    meas.num_sats = static_cast<uint8_t>(actual_sats);

    // Copy satellite data
    for (int i = 0; i < actual_sats; i++) {
        auto& sat = meas.sats[i];
        sat.prn = static_cast<uint8_t>(prns[i]);
        sat.pseudorange = pseudoranges[i];
        sat.doppler = dopplers[i];
        sat.cn0 = cn0s[i];

        // Note: Satellite positions (ECEF) should be computed from ephemeris
        // This requires GNSS navigation message decoding (future enhancement)
        // For now, mark as unavailable
        sat.sat_pos_ecef[0] = 0.0;
        sat.sat_pos_ecef[1] = 0.0;
        sat.sat_pos_ecef[2] = 0.0;
        sat.sat_vel_ecef[0] = 0.0;
        sat.sat_vel_ecef[1] = 0.0;
        sat.sat_vel_ecef[2] = 0.0;
    }

    // Clock bias/drift (TODO: extract from receiver if available)
    meas.clock_bias = 0.0;
    meas.clock_drift = 0.0;

    // Push to queue
    if (!gnss_queue_.push(meas)) {
        overflow_count_.fetch_add(1, std::memory_order_relaxed);
        LOG_WARN("GNSS queue overflow - measurement dropped");
    }
}

JniGnss& JniGnss::instance() {
    static JniGnss inst;
    return inst;
}

}  // namespace fusion

// ===== JNI FUNCTION IMPLEMENTATIONS =====

extern "C" {

JNIEXPORT void JNICALL
Java_com_fusion_GnssService_nativeInit(JNIEnv* /*env*/, jobject /*obj*/) {
    // Instance is created on first access (singleton pattern)
    fusion::JniGnss::instance();
}

JNIEXPORT jboolean JNICALL
Java_com_fusion_GnssService_nativeStart(JNIEnv* /*env*/, jobject /*obj*/) {
    return fusion::JniGnss::instance().start() ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_fusion_GnssService_nativeStop(JNIEnv* /*env*/, jobject /*obj*/) {
    fusion::JniGnss::instance().stop();
}

JNIEXPORT void JNICALL
Java_com_fusion_GnssService_nativeOnMeasurement(
    JNIEnv* env,
    jobject /*obj*/,
    jlong timestamp_ns,
    jint num_sats,
    jintArray prns,
    jdoubleArray pseudoranges,
    jfloatArray dopplers,
    jfloatArray cn0s) {

    if (!prns || !pseudoranges || !dopplers || !cn0s) {
        return;  // Invalid input
    }

    // Get array pointers (no copy)
    jint* prn_data = env->GetIntArrayElements(prns, nullptr);
    jdouble* pr_data = env->GetDoubleArrayElements(pseudoranges, nullptr);
    jfloat* doppler_data = env->GetFloatArrayElements(dopplers, nullptr);
    jfloat* cn0_data = env->GetFloatArrayElements(cn0s, nullptr);

    if (prn_data && pr_data && doppler_data && cn0_data) {
        // Forward to C++ callback
        fusion::JniGnss::instance().onGnssMeasurement(
            timestamp_ns,
            num_sats,
            prn_data,
            pr_data,
            doppler_data,
            cn0_data);
    }

    // Release arrays (no copy back needed - JNI_ABORT)
    if (prn_data) env->ReleaseIntArrayElements(prns, prn_data, JNI_ABORT);
    if (pr_data) env->ReleaseDoubleArrayElements(pseudoranges, pr_data, JNI_ABORT);
    if (doppler_data) env->ReleaseFloatArrayElements(dopplers, doppler_data, JNI_ABORT);
    if (cn0_data) env->ReleaseFloatArrayElements(cn0s, cn0_data, JNI_ABORT);
}

}  // extern "C"

#endif  // __ANDROID__
