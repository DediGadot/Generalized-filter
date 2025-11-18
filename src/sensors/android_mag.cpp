// Android Magnetometer Implementation
#ifdef __ANDROID__

#include "sensors/android_mag.hpp"
#include "utils/logger.hpp"

#include <android/log.h>

namespace fusion {

AndroidMag::AndroidMag(size_t queue_size)
    : sensor_manager_(nullptr),
      event_queue_(nullptr),
      mag_sensor_(nullptr),
      looper_(nullptr),
      mag_queue_(queue_size),
      running_(false),
      overflow_count_(0),
      hard_iron_offset_(Eigen::Vector3f::Zero()),
      soft_iron_matrix_(Eigen::Matrix3f::Identity()) {

    // Get sensor manager instance
    sensor_manager_ = ASensorManager_getInstance();
    if (!sensor_manager_) {
        LOG_ERROR("Failed to get ASensorManager instance");
        return;
    }

    // Get default magnetometer
    mag_sensor_ = ASensorManager_getDefaultSensor(
        sensor_manager_, ASENSOR_TYPE_MAGNETIC_FIELD);
    if (!mag_sensor_) {
        LOG_ERROR("No magnetometer sensor available");
        return;
    }

    LOG_INFO("AndroidMag initialized - Sensor: {}",
             ASensor_getName(mag_sensor_));
}

AndroidMag::~AndroidMag() {
    stop();
}

bool AndroidMag::start(float rate_hz) {
    if (running_.load(std::memory_order_acquire)) {
        LOG_WARN("Magnetometer already running");
        return false;
    }

    if (!sensor_manager_ || !mag_sensor_) {
        LOG_ERROR("Magnetometer not initialized");
        return false;
    }

    // Prepare looper
    looper_ = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    if (!looper_) {
        looper_ = ALooper_forThread();
        if (!looper_) {
            LOG_ERROR("Failed to get/create ALooper");
            return false;
        }
    }

    // Create sensor event queue
    event_queue_ = ASensorManager_createEventQueue(
        sensor_manager_, looper_, ALOOPER_POLL_CALLBACK, sensor_callback, this);
    if (!event_queue_) {
        LOG_ERROR("Failed to create ASensorEventQueue");
        return false;
    }

    // Calculate sampling period
    int32_t period_us = static_cast<int32_t>(1e6f / rate_hz);

    // Enable magnetometer
    int ret = ASensorEventQueue_enableSensor(event_queue_, mag_sensor_);
    if (ret < 0) {
        LOG_ERROR("Failed to enable magnetometer: {}", ret);
        ASensorManager_destroyEventQueue(sensor_manager_, event_queue_);
        event_queue_ = nullptr;
        return false;
    }

    ret = ASensorEventQueue_setEventRate(event_queue_, mag_sensor_, period_us);
    if (ret < 0) {
        LOG_WARN("Failed to set magnetometer rate: {}", ret);
    }

    running_.store(true, std::memory_order_release);

    LOG_INFO("Magnetometer started at {:.1f} Hz ({} µs period)", rate_hz, period_us);
    return true;
}

void AndroidMag::stop() {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }

    running_.store(false, std::memory_order_release);

    if (event_queue_) {
        if (mag_sensor_) {
            ASensorEventQueue_disableSensor(event_queue_, mag_sensor_);
        }
        ASensorManager_destroyEventQueue(sensor_manager_, event_queue_);
        event_queue_ = nullptr;
    }

    LOG_INFO("Magnetometer stopped (overflow count: {})",
             overflow_count_.load());
}

bool AndroidMag::pop(MagSample& sample) {
    return mag_queue_.pop(sample);
}

int AndroidMag::sensor_callback(int /*fd*/, int /*events*/, void* data) {
    auto* self = static_cast<AndroidMag*>(data);
    if (self) {
        self->process_events();
    }
    return 1;
}

void AndroidMag::process_events() {
    ASensorEvent event;

    while (ASensorEventQueue_getEvents(event_queue_, &event, 1) > 0) {
        if (event.type == ASENSOR_TYPE_MAGNETIC_FIELD) {
            // Raw magnetometer reading (µT)
            Eigen::Vector3f raw_mag;
            raw_mag << event.magnetic.x,
                       event.magnetic.y,
                       event.magnetic.z;

            // Apply calibration
            Eigen::Vector3f calibrated = calibrate(raw_mag);

            // Create sample
            MagSample sample;
            sample.timestamp_ns = event.timestamp;
            sample.mag = calibrated;
            sample.temperature = 0.0f;  // Temperature not available in basic API

            // Push to queue
            if (!mag_queue_.push(sample)) {
                overflow_count_.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
}

Eigen::Vector3f AndroidMag::calibrate(const Eigen::Vector3f& raw) const {
    // Apply calibration: corrected = S * (raw - b)
    // where S is soft iron matrix, b is hard iron offset
    return soft_iron_matrix_ * (raw - hard_iron_offset_);
}

}  // namespace fusion

#endif  // __ANDROID__
