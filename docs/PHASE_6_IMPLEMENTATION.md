# Phase 6: Android Service Integration - Implementation Plan

**Date**: 2025-11-18
**Status**: In Progress
**Prerequisites**: ✅ Phases 0-5 Complete, ✅ Optimization Bug Fixed

---

## Executive Summary

Phase 6 transforms the validated filter algorithms (Phases 0-5) into a production-ready Android service. This phase focuses on:

1. **Real-time thread management** - SCHED_FIFO priority for deterministic performance
2. **Android lifecycle integration** - Foreground service that survives app backgrounding
3. **Power optimization** - Adaptive rate (50-200 Hz) based on motion detection
4. **Production monitoring** - Thermal, battery, CPU profiling

**Current Performance Baseline**:
- ✅ Filter cycle: 33.971 µs with -O3 optimization
- ✅ Real-time capability: 2943x faster than required (10 Hz)
- ✅ All integration tests passing

**Phase 6 Goal**: Deploy filter as Android service with <10% CPU usage, <50mA power draw, <5ms end-to-end latency.

---

## Architecture: Production Deployment Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                    ANDROID APPLICATION LAYER                     │
│                   (Your AR Glasses App)                          │
│                                                                  │
│  getPose() ────────────────┐                                    │
│  getVelocity() ────────────┤  IPC (Binder)                      │
│  getCovariance() ──────────┘                                    │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│              FUSIONSERVICE (Android Foreground Service)          │
│                      FusionService.java                          │
│                                                                  │
│  • Lifecycle management (onCreate/onDestroy)                    │
│  • Foreground notification (keeps service alive)                │
│  • Binder interface (IFusionService.aidl)                       │
│  • Battery/thermal monitoring                                   │
│                                                                  │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │           JNI BRIDGE (fusion_jni.cpp)                     │ │
│  │                                                           │ │
│  │  Java_...FusionService_nativeInit()                      │ │
│  │  Java_...FusionService_nativeStart()                     │ │
│  │  Java_...FusionService_nativeStop()                      │ │
│  │  Java_...FusionService_nativeGetPose(...)               │ │
│  └───────────────────────────┬───────────────────────────────┘ │
└────────────────────────────────┼─────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│           C++ FUSION THREAD (fusion_thread.cpp)                  │
│                   pthread with SCHED_FIFO                        │
│                   Pinned to big core (cortex-a76)                │
│                                                                  │
│  while (running_) {                                             │
│      // Adaptive timing: 50Hz stationary, 200Hz moving          │
│      wait_for_next_epoch();                                     │
│                                                                  │
│      // Step 1: Preintegrate IMU                                │
│      preintegrate_imu(&imu_queue_);                            │
│                                                                  │
│      // Step 2: EKF Prediction                                  │
│      ekf_state_.predict(preint_result_);                        │
│                                                                  │
│      // Step 3: Sensor updates (mag, GNSS when available)       │
│      process_sensor_updates();                                  │
│                                                                  │
│      // Step 4: Output fused state                              │
│      publish_state();  // <-- JNI callback or lock-free queue   │
│                                                                  │
│      // Step 5: Monitor health (thermal, divergence)            │
│      check_filter_health();                                     │
│  }                                                               │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐│
│  │  SENSOR QUEUES (lock-free SPSC)                           ││
│  │  • IMU queue (512 samples)                                ││
│  │  • Magnetometer queue (64 samples)                        ││
│  │  • GNSS queue (16 fixes)                                  ││
│  └────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
                                 ▲
                                 │
    ┌────────────────────────────┼────────────────────────────┐
    │                            │                            │
    ▼                            ▼                            ▼
┌─────────┐              ┌─────────────┐            ┌──────────────┐
│ IMU     │              │ Magnetometer│            │ GNSS         │
│ 200 Hz  │              │ 50 Hz       │            │ 1-10 Hz      │
└─────────┘              └─────────────┘            └──────────────┘
ASensorManager           ASensorManager         LocationManager
(NDK Direct)             (NDK Direct)           (JNI Callback)
```

---

## Component 1: Fusion Thread (C++)

### File: `src/service/fusion_thread.cpp`

**Purpose**: Real-time fusion loop with adaptive rate and thread affinity.

**Key Features**:
1. **pthread with SCHED_FIFO priority** - Linux real-time scheduling
2. **CPU affinity** - Pin to big core (cortex-a76 on Snapdragon AR1)
3. **Adaptive rate** - 50 Hz stationary, 100-200 Hz during motion
4. **Lock-free queues** - SPSC queues for sensor data
5. **Health monitoring** - Thermal, divergence, sensor dropout detection

**Thread Timing**:
```
Stationary:  50 Hz  = 20ms period (600µs filter cycle = 3% duty cycle)
Moving:     100 Hz  = 10ms period (600µs filter cycle = 6% duty cycle)
Aggressive: 200 Hz  = 5ms period  (600µs filter cycle = 12% duty cycle)
```

**Power Budget**:
- Target: <10% CPU average = <50mA additional power
- Achievable with adaptive rate (most time stationary at 50 Hz)

### Implementation Strategy

```cpp
class FusionThread {
public:
    // Lifecycle
    bool start();
    void stop();
    bool is_running() const;

    // Configuration
    void set_adaptive_rate(bool enabled);
    void set_base_rate_hz(double hz);
    void set_cpu_affinity(int core_id);

    // State access (thread-safe)
    bool get_current_pose(Pose& pose) const;
    bool get_current_state(FusedState& state) const;

    // Diagnostics
    ThreadStats get_stats() const;

private:
    void run();  // Main fusion loop

    // Adaptive rate control
    double compute_adaptive_rate();
    bool is_stationary() const;

    // Health monitoring
    void check_thermal_throttling();
    void check_filter_divergence();
    void check_sensor_health();

    // Thread management
    pthread_t thread_;
    std::atomic<bool> running_;

    // Filter components
    EkfState ekf_state_;
    ImuPreintegration imu_preint_;
    MagUpdate mag_update_;

    // Sensor queues (populated by JNI/NDK)
    SPSCQueue<ImuSample> imu_queue_;
    SPSCQueue<MagSample> mag_queue_;

    // Output state (lock-free publish)
    std::atomic<FusedState> current_state_;

    // Configuration
    double base_rate_hz_{100.0};
    bool adaptive_rate_{true};
    int cpu_affinity_{-1};  // -1 = no affinity

    // Statistics
    std::atomic<uint64_t> cycle_count_{0};
    std::atomic<uint64_t> total_cycles_us_{0};
    std::atomic<uint32_t> thermal_throttle_count_{0};
};
```

---

## Component 2: Android Foreground Service (Java)

### File: `android/FusionService.java`

**Purpose**: Android lifecycle management for fusion filter.

**Key Features**:
1. **Foreground service** - Survives app backgrounding, screen off
2. **Persistent notification** - Required for foreground service
3. **Binder interface** - IPC for apps to query pose
4. **Battery monitoring** - Log power consumption
5. **Lifecycle hooks** - Proper start/stop of C++ thread

**Service Lifecycle**:
```
onCreate()      → nativeInit()          (Initialize C++ filter)
onStartCommand()→ nativeStart()         (Start fusion thread)
getPose()       → nativeGetPose()       (Query current state via JNI)
onDestroy()     → nativeStop()          (Clean shutdown)
```

**Foreground Notification**:
- Required for Android 12+ foreground services
- Shows "AR Tracking Active" with battery/CPU stats
- Allows user to stop service

### Implementation Strategy

```java
public class FusionService extends Service {
    private static final String TAG = "FusionService";
    private static final int NOTIFICATION_ID = 1001;

    // Binder for IPC
    private final IBinder binder = new FusionBinder();

    public class FusionBinder extends Binder {
        public FusionService getService() {
            return FusionService.this;
        }
    }

    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "FusionService onCreate");

        // Initialize C++ filter
        if (!nativeInit()) {
            Log.e(TAG, "Failed to initialize native filter");
            stopSelf();
            return;
        }
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        Log.i(TAG, "FusionService starting");

        // Create foreground notification
        startForeground(NOTIFICATION_ID, createNotification());

        // Start fusion thread
        if (!nativeStart()) {
            Log.e(TAG, "Failed to start fusion thread");
            stopSelf();
            return START_NOT_STICKY;
        }

        // Start battery/thermal monitoring
        startMonitoring();

        return START_STICKY;  // Restart if killed
    }

    @Override
    public void onDestroy() {
        Log.i(TAG, "FusionService stopping");

        // Stop fusion thread
        nativeStop();

        // Stop monitoring
        stopMonitoring();

        super.onDestroy();
    }

    // === Public API (called via Binder) ===

    public Pose getCurrentPose() {
        return nativeGetPose();
    }

    public FusedState getCurrentState() {
        return nativeGetState();
    }

    public FilterStats getStats() {
        return nativeGetStats();
    }

    // === JNI Native Methods ===

    private native boolean nativeInit();
    private native boolean nativeStart();
    private native void nativeStop();
    private native Pose nativeGetPose();
    private native FusedState nativeGetState();
    private native FilterStats nativeGetStats();

    static {
        System.loadLibrary("fusion");
    }
}
```

---

## Component 3: JNI Bridge (C++)

### File: `src/service/fusion_jni.cpp`

**Purpose**: Java ↔ C++ interface for FusionService.

**Key Features**:
1. **Lifecycle management** - Init, start, stop from Java
2. **State queries** - Get pose, velocity, covariance
3. **Statistics** - CPU, timing, health metrics
4. **Minimal overhead** - <1µs per JNI call

**JNI Function Signatures**:

```cpp
extern "C" {

// Lifecycle
JNIEXPORT jboolean JNICALL
Java_com_fusion_FusionService_nativeInit(JNIEnv* env, jobject thiz);

JNIEXPORT jboolean JNICALL
Java_com_fusion_FusionService_nativeStart(JNIEnv* env, jobject thiz);

JNIEXPORT void JNICALL
Java_com_fusion_FusionService_nativeStop(JNIEnv* env, jobject thiz);

// State queries
JNIEXPORT jobject JNICALL
Java_com_fusion_FusionService_nativeGetPose(JNIEnv* env, jobject thiz);

JNIEXPORT jobject JNICALL
Java_com_fusion_FusionService_nativeGetState(JNIEnv* env, jobject thiz);

JNIEXPORT jobject JNICALL
Java_com_fusion_FusionService_nativeGetStats(JNIEnv* env, jobject thiz);

}  // extern "C"
```

**Java Object Creation** (Pose, FusedState, etc.):
- Cache jclass/jmethodID for object creation (avoid repeated FindClass)
- Use NewObject() to create Java objects from C++ data
- Minimal memory allocation (stack-based where possible)

---

## Component 4: Power Management

### Adaptive Rate Control

**Motion Detection**:
```cpp
bool FusionThread::is_stationary() const {
    // Check IMU variance over last 1 second
    Vector3f accel_var = compute_accel_variance(last_1s_samples_);
    Vector3f gyro_var = compute_gyro_variance(last_1s_samples_);

    // Thresholds (empirically tuned)
    const float ACCEL_STATIONARY_THRESHOLD = 0.05f;  // m/s²
    const float GYRO_STATIONARY_THRESHOLD = 0.01f;   // rad/s

    return (accel_var.norm() < ACCEL_STATIONARY_THRESHOLD &&
            gyro_var.norm() < GYRO_STATIONARY_THRESHOLD);
}

double FusionThread::compute_adaptive_rate() {
    if (!adaptive_rate_) {
        return base_rate_hz_;
    }

    if (is_stationary()) {
        return 50.0;  // Stationary: 50 Hz
    } else {
        // Moving: scale with motion magnitude
        float motion_mag = compute_motion_magnitude();
        return std::clamp(50.0 + motion_mag * 150.0, 50.0, 200.0);
    }
}
```

### Thermal Monitoring

**CPU Temperature Check**:
```cpp
void FusionThread::check_thermal_throttling() {
    // Read CPU temperature from /sys/class/thermal/thermal_zone0/temp
    int temp_milliC = read_cpu_temperature();
    float temp_C = temp_milliC / 1000.0f;

    const float THROTTLE_THRESHOLD = 70.0f;  // °C
    const float CRITICAL_THRESHOLD = 80.0f;

    if (temp_C > CRITICAL_THRESHOLD) {
        // Critical: reduce to minimum rate
        base_rate_hz_ = 25.0;
        thermal_throttle_count_++;
        LOG_WARN("Thermal throttle: %.1f°C, reducing to 25 Hz", temp_C);
    } else if (temp_C > THROTTLE_THRESHOLD) {
        // Moderate: reduce by 50%
        base_rate_hz_ = std::max(50.0, base_rate_hz_ * 0.5);
        LOG_INFO("Thermal warning: %.1f°C, reducing rate", temp_C);
    }
}
```

---

## Component 5: Android Manifest

### File: `android/AndroidManifest.xml`

**Required Permissions**:

```xml
<?xml version="1.0" encoding="utf-8"?>
<manifest xmlns:android="http://schemas.android.com/apk/res/android"
    package="com.fusion">

    <!-- Sensor access -->
    <uses-permission android:name="android.permission.HIGH_SAMPLING_RATE_SENSORS" />

    <!-- Location access -->
    <uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />
    <uses-permission android:name="android.permission.ACCESS_COARSE_LOCATION" />

    <!-- Foreground service (Android 9+) -->
    <uses-permission android:name="android.permission.FOREGROUND_SERVICE" />
    <uses-permission android:name="android.permission.FOREGROUND_SERVICE_LOCATION" />

    <!-- Wake lock (keep running when screen off) -->
    <uses-permission android:name="android.permission.WAKE_LOCK" />

    <!-- Real-time priority (requires system signature) -->
    <!-- <uses-permission android:name="android.permission.REAL_TIME_PRIORITY" /> -->

    <application
        android:label="Fusion Filter"
        android:theme="@style/AppTheme">

        <!-- Foreground Service -->
        <service
            android:name=".FusionService"
            android:enabled="true"
            android:exported="false"
            android:foregroundServiceType="location" />

    </application>
</manifest>
```

**Note**: `REAL_TIME_PRIORITY` permission requires system-level signature. For non-system apps, use regular priority and rely on Android scheduler.

---

## Performance Targets & Validation

### CPU Usage Target: <10% Average

**Calculation**:
- Snapdragon AR1 Gen 1: Cortex-A76 @ 2.0 GHz
- Filter cycle: 33.971 µs
- At 100 Hz: 33.971 µs × 100 = 3.4 ms/sec = 0.34% duty cycle
- At 50 Hz (adaptive): 33.971 µs × 50 = 1.7 ms/sec = 0.17% duty cycle

**Reality Check**: Include JNI overhead, queue management, monitoring:
- Estimated total: ~1-2% CPU at 50 Hz, ~3-5% at 100 Hz
- ✅ Well below 10% target

### Power Target: <50mA Additional Draw

**Baseline Power**:
- Snapdragon AR1 idle: ~150mA
- IMU sampling (200 Hz): ~5mA
- CPU at 1-2% (50 Hz fusion): ~10-20mA
- Display + OS: ~300mA (typical)

**Fusion Filter Additional**:
- IMU already running: 0mA (shared with system)
- CPU for fusion: ~10-20mA
- Wake lock overhead: ~5mA

**Estimated Total**: 15-25mA additional (✅ below 50mA target)

### Latency Target: <5ms End-to-End

**Latency Breakdown**:
1. Sensor ISR → Queue: <100µs (lock-free push)
2. Queue → Fusion thread wake: <500µs (SCHED_FIFO priority)
3. Fusion cycle: 34µs
4. JNI callback: <1µs
5. Total: <1ms (✅ well below 5ms target)

---

## Implementation Order

1. ✅ **Fusion Thread (C++)** - Core real-time loop
2. **JNI Bridge** - Expose C++ to Java
3. **Android Service** - Lifecycle + foreground notification
4. **Android Manifest** - Permissions
5. **Power Management** - Adaptive rate + thermal
6. **Testing** - Validate on device

---

## Testing Strategy

### Unit Tests (Desktop)
- Fusion thread lifecycle (start/stop/restart)
- Adaptive rate logic (stationary detection)
- Thread safety (concurrent state access)

### Integration Tests (Android Device)
1. **Background Operation**:
   - Start service → press home → verify fusion continues
   - Lock screen → verify fusion continues
   - Kill app → verify service restarts (START_STICKY)

2. **Performance**:
   - CPU usage (Android Studio Profiler)
   - Power draw (Battery Historian)
   - Thermal behavior (stress test for 10 minutes)

3. **Latency**:
   - Timestamp sensor samples → measure output latency
   - Target: <5ms end-to-end

4. **Accuracy** (compared to ground truth):
   - Walk straight 10m → compare to measured distance
   - Rotate 360° → compare to integrated heading
   - Figure-8 pattern → validate drift correction

---

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Android kills service (battery optimization) | Medium | High | Foreground service + notification |
| Real-time priority denied (non-system app) | High | Medium | Graceful fallback to normal priority |
| Thermal throttling on sustained use | Medium | Medium | Adaptive rate + thermal monitoring |
| JNI overhead too high | Low | Medium | Cache jclass/jmethodID, minimize calls |
| Filter divergence in production | Low | High | Health monitoring + auto-reinit |

---

## Next Steps

1. Implement `FusionThread` class
2. Implement JNI bridge
3. Implement `FusionService.java`
4. Create Android manifest
5. Build and deploy to test device
6. Profile and optimize

**Estimated Implementation Time**: 2-3 days for full Phase 6.

---

**Status**: ⏳ Ready to implement (optimizer bug fixed, Phases 0-5 complete)
