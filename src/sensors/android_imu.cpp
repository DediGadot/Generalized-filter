// Android IMU Sensor Implementation
#ifdef __ANDROID__

#include "sensors/android_imu.hpp"
#include "utils/logger.hpp"

#include <android/log.h>
#include <cstring>

namespace fusion {

AndroidImu::AndroidImu(size_t queue_size)
    : sensor_manager_(nullptr),
      event_queue_(nullptr),
      accel_sensor_(nullptr),
      gyro_sensor_(nullptr),
      looper_(nullptr),
      imu_queue_(queue_size),
      running_(false),
      overflow_count_(0) {

    // Initialize buffer
    std::memset(&buffer_, 0, sizeof(buffer_));
    buffer_.has_accel = false;
    buffer_.has_gyro = false;

    // Get sensor manager instance
    sensor_manager_ = ASensorManager_getInstance();
    if (!sensor_manager_) {
        LOG_ERROR("Failed to get ASensorManager instance");
        return;
    }

    // Get default accelerometer
    accel_sensor_ = ASensorManager_getDefaultSensor(
        sensor_manager_, ASENSOR_TYPE_ACCELEROMETER);
    if (!accel_sensor_) {
        LOG_ERROR("No accelerometer sensor available");
        return;
    }

    // Get default gyroscope
    gyro_sensor_ = ASensorManager_getDefaultSensor(
        sensor_manager_, ASENSOR_TYPE_GYROSCOPE);
    if (!gyro_sensor_) {
        LOG_ERROR("No gyroscope sensor available");
        return;
    }

    LOG_INFO("AndroidImu initialized - Accel: {}, Gyro: {}",
             ASensor_getName(accel_sensor_),
             ASensor_getName(gyro_sensor_));
}

AndroidImu::~AndroidImu() {
    stop();
}

bool AndroidImu::start(float rate_hz) {
    if (running_.load(std::memory_order_acquire)) {
        LOG_WARN("IMU already running");
        return false;
    }

    if (!sensor_manager_ || !accel_sensor_ || !gyro_sensor_) {
        LOG_ERROR("Sensors not initialized");
        return false;
    }

    // Prepare looper (use current thread's looper or create one)
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

    // Calculate sampling period in microseconds
    int32_t period_us = static_cast<int32_t>(1e6f / rate_hz);

    // Enable accelerometer
    int ret = ASensorEventQueue_enableSensor(event_queue_, accel_sensor_);
    if (ret < 0) {
        LOG_ERROR("Failed to enable accelerometer: {}", ret);
        ASensorManager_destroyEventQueue(sensor_manager_, event_queue_);
        event_queue_ = nullptr;
        return false;
    }

    ret = ASensorEventQueue_setEventRate(event_queue_, accel_sensor_, period_us);
    if (ret < 0) {
        LOG_WARN("Failed to set accelerometer rate: {}", ret);
        // Continue anyway - will use default rate
    }

    // Enable gyroscope
    ret = ASensorEventQueue_enableSensor(event_queue_, gyro_sensor_);
    if (ret < 0) {
        LOG_ERROR("Failed to enable gyroscope: {}", ret);
        ASensorEventQueue_disableSensor(event_queue_, accel_sensor_);
        ASensorManager_destroyEventQueue(sensor_manager_, event_queue_);
        event_queue_ = nullptr;
        return false;
    }

    ret = ASensorEventQueue_setEventRate(event_queue_, gyro_sensor_, period_us);
    if (ret < 0) {
        LOG_WARN("Failed to set gyroscope rate: {}", ret);
        // Continue anyway
    }

    running_.store(true, std::memory_order_release);

    LOG_INFO("IMU started at {:.1f} Hz ({} µs period)", rate_hz, period_us);
    return true;
}

void AndroidImu::stop() {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }

    running_.store(false, std::memory_order_release);

    if (event_queue_) {
        // Disable sensors
        if (accel_sensor_) {
            ASensorEventQueue_disableSensor(event_queue_, accel_sensor_);
        }
        if (gyro_sensor_) {
            ASensorEventQueue_disableSensor(event_queue_, gyro_sensor_);
        }

        // Destroy event queue
        ASensorManager_destroyEventQueue(sensor_manager_, event_queue_);
        event_queue_ = nullptr;
    }

    LOG_INFO("IMU stopped (overflow count: {})", overflow_count_.load());
}

bool AndroidImu::pop(ImuSample& sample) {
    return imu_queue_.pop(sample);
}

int AndroidImu::sensor_callback(int /*fd*/, int /*events*/, void* data) {
    auto* self = static_cast<AndroidImu*>(data);
    if (self) {
        self->process_events();
    }
    return 1;  // Continue receiving events
}

void AndroidImu::process_events() {
    ASensorEvent event;

    // Process all available events
    while (ASensorEventQueue_getEvents(event_queue_, &event, 1) > 0) {
        if (event.type == ASENSOR_TYPE_ACCELEROMETER) {
            // Store accelerometer data
            buffer_.timestamp_ns = event.timestamp;
            buffer_.accel << event.acceleration.x,
                             event.acceleration.y,
                             event.acceleration.z;
            buffer_.has_accel = true;

        } else if (event.type == ASENSOR_TYPE_GYROSCOPE) {
            // Store gyroscope data
            buffer_.timestamp_ns = event.timestamp;
            buffer_.gyro << event.vector.x,
                            event.vector.y,
                            event.vector.z;
            buffer_.has_gyro = true;
        }

        // If we have both accel and gyro, create IMU sample
        if (buffer_.has_accel && buffer_.has_gyro) {
            ImuSample sample;
            sample.timestamp_ns = buffer_.timestamp_ns;
            sample.gyro = buffer_.gyro;
            sample.accel = buffer_.accel;
            sample.flags = 0;

            // Push to queue
            if (!imu_queue_.push(sample)) {
                // Queue overflow
                overflow_count_.fetch_add(1, std::memory_order_relaxed);
            }

            // Reset buffer
            buffer_.has_accel = false;
            buffer_.has_gyro = false;
        }
    }
}

}  // namespace fusion

#endif  // __ANDROID__
