# Transition Proposal: From Tightly-Coupled GNSS to Android Fused Location

**Date**: 2025-11-18
**Status**: Proposal
**Impact**: Architecture change from v1.5 plan

---

## Executive Summary

This document proposes transitioning from the planned **tightly-coupled GNSS** integration (raw pseudoranges) to **Android Fused Location** integration (pre-computed position solution). This significantly simplifies implementation while improving real-world robustness by leveraging Android's battle-tested location infrastructure.

### Current State (v1.0)
- GNSS infrastructure prepared but not implemented
- `GnssMeasurement` struct designed for tightly-coupled raw pseudorange fusion
- Planning individual satellite measurement updates (8-32 satellites per epoch)

### Proposed Change
- Replace raw GNSS with Android `FusedLocationProvider` API
- Integrate pre-computed position solutions as 3D position measurements
- Reuse existing measurement update framework (identical pattern to magnetometer)

### Key Benefits
1. **Simplicity**: ~80% less code complexity (no satellite ephemeris, no pseudorange modeling)
2. **Robustness**: Leverage Android's multi-source fusion (GPS + WiFi + cell + BLE)
3. **Indoor/Urban**: Automatic WiFi/cell fallback when GPS unavailable
4. **Battery**: Android optimizes sensor usage across all apps
5. **Proven**: Battle-tested by billions of Android devices

### Trade-offs
1. **Observability**: Lose direct control over satellite weighting and outlier rejection
2. **Latency**: Android location has ~100-300ms internal latency
3. **Accuracy**: Slightly lower peak accuracy vs. raw GNSS in open sky (5-10m vs. 2-5m)
4. **States**: Cannot estimate receiver clock bias/drift (simpler 15D state, not 17D)

---

## 1. Architecture Comparison

### Current v1.5 Plan: Tightly-Coupled GNSS

```
┌─────────────────────────────────────────────────────────────────┐
│ Android LocationManager.registerGnssMeasurementsCallback()      │
│ Raw GNSS: pseudoranges, Doppler, CN0, satellite positions      │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ JNI Bridge (jni_gnss.cpp)                                       │
│ - Marshal satellite data from Java arrays                       │
│ - Convert ECEF satellite positions                              │
│ - Extract clock bias/drift estimates                            │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Lock-free SPSC Queue<GnssMeasurement> (capacity: 16)            │
│ - Each measurement: 8-32 satellites × ~100 bytes = ~3 KB       │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Fusion Thread: GnssUpdate Class                                 │
│                                                                  │
│ For each satellite (8-32 iterations):                           │
│   1. Predict pseudorange: ||p_ecef - sat_pos_ecef|| + c·bias   │
│   2. Compute innovation: measured - predicted                   │
│   3. Build Jacobian: ∂pseudorange/∂position, ∂/∂clock_bias     │
│   4. Weight by CN0 (signal quality)                             │
│   5. Elevation masking (reject low satellites)                  │
│   6. Chi-square gating (outlier rejection)                      │
│   7. Adaptive noise (multipath/urban canyon detection)          │
│                                                                  │
│ Measurement dimension: N_sats (8-32D measurement)               │
│ State dimension: 17D (position, velocity, attitude,             │
│                       biases, clock_bias, clock_drift)          │
│                                                                  │
│ Computational cost: ~50-100 µs per update                       │
└─────────────────────────────────────────────────────────────────┘

Complexity:
- ~800 lines of code (GnssUpdate class, satellite geometry, ephemeris)
- Requires understanding of GNSS pseudorange model
- Need atmospheric corrections (ionosphere/troposphere)
- Must handle satellite geometry (GDOP, elevation masks)
- Clock bias/drift states add 2D to state vector
```

### Proposed: Android Fused Location

```
┌─────────────────────────────────────────────────────────────────┐
│ Android FusedLocationProviderClient.requestLocationUpdates()   │
│ Pre-fused position: GPS + WiFi + Cell + BLE                    │
│ Output: lat/lon/alt + accuracy estimate + timestamp            │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ JNI Bridge (jni_location.cpp)                                   │
│ - Marshal simple Location object from Java                      │
│ - Convert WGS84 (lat/lon/alt) → NED frame (local tangent)      │
│ - Extract accuracy → measurement noise R                        │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Lock-free SPSC Queue<LocationMeasurement> (capacity: 16)        │
│ - Each measurement: ~64 bytes (position + accuracy + metadata) │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│ Fusion Thread: LocationUpdate Class                             │
│                                                                  │
│ Single iteration (simplified):                                  │
│   1. Convert WGS84 → NED (if not already converted)            │
│   2. Predict position: p_n (from nominal state)                 │
│   3. Compute innovation: z_measured - p_n                       │
│   4. Build Jacobian: H = [0, 0, I_3x3, 0, 0] (identity on pos) │
│   5. Adaptive noise: R = diag(accuracy²) × adaptive_factor     │
│   6. Chi-square gating (outlier rejection)                      │
│   7. Generic update                                             │
│                                                                  │
│ Measurement dimension: 3D (North, East, Down position)          │
│ State dimension: 15D (unchanged - no clock states needed)      │
│                                                                  │
│ Computational cost: ~5-10 µs per update                         │
└─────────────────────────────────────────────────────────────────┘

Complexity:
- ~200 lines of code (LocationUpdate class, WGS84→NED conversion)
- Simple 3D position measurement (follows magnetometer pattern)
- No satellite-level complexity
- Android handles multi-source fusion internally
- No additional states required
```

---

## 2. Detailed Technical Design

### 2.1 Data Structures

#### Replace GnssMeasurement with LocationMeasurement

**Current (sensor_types.hpp, lines 84-97)**:
```cpp
struct GnssMeasurement {
    timestamp_t timestamp_ns;
    uint8_t num_sats;
    GnssSatellite sats[MAX_GNSS_SATS];  // Complex satellite data
    double clock_bias;
    double clock_drift;
};
```

**Proposed**:
```cpp
/**
 * @brief Android Fused Location measurement
 *
 * Pre-computed position from FusedLocationProvider API.
 * Combines GPS, WiFi, cell towers, and BLE for robust positioning.
 *
 * Size: 64 bytes (cache-friendly)
 * Frequency: 0.5-5 Hz (configurable via Priority)
 */
struct LocationMeasurement {
    timestamp_t timestamp_ns;       ///< Measurement timestamp

    // Position in WGS84 geodetic coordinates
    double latitude_deg;            ///< Latitude [-90, 90] degrees
    double longitude_deg;           ///< Longitude [-180, 180] degrees
    double altitude_m;              ///< Altitude above WGS84 ellipsoid [m]

    // Position in local NED frame (computed on receipt)
    Vector3d position_ned;          ///< Position [North, East, Down] in meters

    // Uncertainty estimates (from Android)
    float horizontal_accuracy_m;    ///< Horizontal accuracy (68% confidence) [m]
    float vertical_accuracy_m;      ///< Vertical accuracy (68% confidence) [m]
    float speed_accuracy_mps;       ///< Speed accuracy [m/s] (optional)

    // Metadata
    uint8_t provider_flags;         ///< Bit flags: GPS=1, WiFi=2, Cell=4, BLE=8
    uint8_t num_satellites;         ///< Number of GPS satellites used (if available)

    LocationMeasurement()
        : timestamp_ns(0),
          latitude_deg(0.0), longitude_deg(0.0), altitude_m(0.0),
          position_ned(Vector3d::Zero()),
          horizontal_accuracy_m(999.0f),
          vertical_accuracy_m(999.0f),
          speed_accuracy_mps(999.0f),
          provider_flags(0),
          num_satellites(0) {}
};
```

**Size Comparison**:
- `GnssMeasurement`: ~3 KB (32 satellites × 100 bytes each)
- `LocationMeasurement`: 64 bytes (47× smaller!)

---

### 2.2 JNI Bridge

#### New File: `src/sensors/jni_location.hpp`

```cpp
// JNI Bridge for Android Fused Location
//
// Purpose: Receive pre-computed location from Android FusedLocationProvider
// Documentation: https://developer.android.com/training/location
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
// Sample Usage (Java):
//   LocationService service = new LocationService(context);
//   service.startLocationUpdates(Priority.PRIORITY_HIGH_ACCURACY, 1000);

#pragma once

#ifdef __ANDROID__

#include <jni.h>
#include <atomic>
#include <memory>

#include "core/lockfree_queue.hpp"
#include "core/sensor_types.hpp"

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

class JniLocation {
public:
    explicit JniLocation(size_t queue_size = 16);
    ~JniLocation();

    // Non-copyable, non-movable
    JniLocation(const JniLocation&) = delete;
    JniLocation& operator=(const JniLocation&) = delete;

    /**
     * @brief Start location updates
     *
     * @param priority Location priority (accuracy vs battery)
     * @param interval_sec Update interval in seconds (0.2 - 10.0)
     * @return true if started successfully
     */
    bool start(LocationPriority priority = LocationPriority::HIGH_ACCURACY,
               double interval_sec = 1.0);

    void stop();
    bool is_running() const { return running_.load(std::memory_order_acquire); }

    /**
     * @brief Pop location measurement from queue (non-blocking)
     * @return true if measurement retrieved, false if queue empty
     */
    bool pop(LocationMeasurement& meas);

    // Queue statistics
    size_t queue_size() const { return location_queue_.size(); }
    size_t queue_capacity() const { return location_queue_.capacity(); }
    uint64_t get_overflow_count() const {
        return overflow_count_.load(std::memory_order_acquire);
    }

    // === JNI CALLBACK INTERFACE ===
    // Called from Java via JNI when location arrives
    void onLocationUpdate(
        int64_t timestamp_ns,
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        float horizontal_accuracy_m,
        float vertical_accuracy_m,
        float speed_mps,
        float speed_accuracy_mps,
        int provider_flags);

    static JniLocation& instance();

private:
    LockFreeQueue<LocationMeasurement> location_queue_;
    std::atomic<bool> running_;
    std::atomic<uint64_t> overflow_count_;

    // Reference point for NED frame conversion
    // Set on first location fix
    bool reference_set_;
    double ref_lat_deg_;
    double ref_lon_deg_;
    double ref_alt_m_;

    /**
     * @brief Convert WGS84 to local NED frame
     *
     * Uses simple tangent plane approximation (valid for <10km radius)
     * For production, use full geodetic conversion if needed
     */
    Vector3d wgs84_to_ned(double lat_deg, double lon_deg, double alt_m) const;
};

}  // namespace fusion

// === JNI FUNCTIONS ===

extern "C" {

JNIEXPORT void JNICALL
Java_com_fusion_LocationService_nativeInit(JNIEnv* env, jobject obj);

JNIEXPORT jboolean JNICALL
Java_com_fusion_LocationService_nativeStart(
    JNIEnv* env, jobject obj, jint priority, jdouble interval_sec);

JNIEXPORT void JNICALL
Java_com_fusion_LocationService_nativeStop(JNIEnv* env, jobject obj);

// Callback when location arrives
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
```

---

### 2.3 Measurement Update Class

#### New File: `src/filter/location_update.hpp`

```cpp
// Location Measurement Update
//
// Purpose: Provides position correction using Android Fused Location
// Reference: Follows magnetometer update pattern (mag_update.hpp)
//
// Key Features:
// - 3DOF position measurement (North, East, Down)
// - Adaptive noise based on Android accuracy estimates
// - Outlier rejection via chi-square gating
// - Simple Jacobian (identity matrix for position block)
//
// Measurement Model:
//   z = h(x) + v
//   where:
//     z = measured position in NED frame [m]
//     h(x) = p_n = predicted position from nominal state
//     v ~ N(0, R) = measurement noise (from accuracy estimate)
//
// Linearized Measurement Model:
//   δz = H * δx + v
//   where:
//     H = ∂h/∂x = measurement Jacobian (3 × 15)
//     For position: only depends on position error
//     ∂h/∂δp = I_3x3 (identity)
//     ∂h/∂(other states) = 0
//
// Adaptive Noise:
//   - Use Android's accuracy estimate as base noise
//   - If accuracy is poor (>50m), increase R to reduce trust
//   - If multiple consecutive outliers, mark location as unreliable
//
// Sample Usage:
//   LocationUpdate location_update;
//
//   // Get location from Android
//   LocationMeasurement loc = android_location.get();
//
//   // Perform update (with adaptive noise and outlier rejection)
//   if (location_update.update(state, loc)) {
//       // Update accepted
//       state.inject_error(state.error_state());
//       state.reset_error();
//   } else {
//       // Update rejected (outlier or poor accuracy)
//   }
//
// Expected Output:
//   - Position drift corrected (aligned with GPS/WiFi/cell fusion)
//   - Position covariance reduced
//   - Outliers/poor fixes rejected automatically

#pragma once

#include "ekf_state.hpp"
#include "ekf_update.hpp"
#include "core/types.hpp"
#include "core/sensor_types.hpp"

#include <Eigen/Dense>

namespace fusion {

/**
 * @brief Location measurement update
 *
 * Provides position correction using Android's fused location
 * with adaptive noise rejection based on accuracy estimates
 */
class LocationUpdate {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor
     *
     * @param adaptive_noise_factor Factor for adaptive noise scaling (default: 2.0)
     * @param max_horizontal_error Maximum acceptable horizontal error [m] (default: 100.0)
     */
    LocationUpdate(double adaptive_noise_factor = 2.0,
                   double max_horizontal_error = 100.0);

    /**
     * @brief Update EKF state with location measurement
     *
     * Performs full measurement update:
     * 1. Extract position in NED frame
     * 2. Predict measurement h(x) = p_n
     * 3. Compute innovation y = z - h(x)
     * 4. Compute Jacobian H (identity for position)
     * 5. Adaptive noise (scale by accuracy estimate)
     * 6. Generic update with chi-square gating
     *
     * @param state EKF state to update (modified in-place)
     * @param location_meas Measured location from Android
     * @param enable_gating If true, reject outliers via chi-square test (default: true)
     * @return true if update accepted, false if rejected as outlier/poor accuracy
     */
    bool update(
        EkfState& state,
        const LocationMeasurement& location_meas,
        bool enable_gating = true);

    /**
     * @brief Check if location quality is acceptable
     *
     * Rejects location if:
     * - Horizontal accuracy > max_horizontal_error
     * - Vertical accuracy > 2 × max_horizontal_error
     * - Provider flags indicate no GPS/WiFi (cell-only)
     *
     * @param location_meas Location measurement
     * @return true if quality is acceptable
     */
    bool is_location_valid(const LocationMeasurement& location_meas) const;

    /**
     * @brief Get last innovation (for diagnostics)
     *
     * @return Last innovation vector [m] (NED frame)
     */
    Vector3d get_last_innovation() const { return last_innovation_; }

    /**
     * @brief Get last adaptive noise factor (for diagnostics)
     *
     * @return Last adaptive noise scaling factor (1.0 = nominal, >1.0 = increased)
     */
    double get_last_adaptive_factor() const { return last_adaptive_factor_; }

private:
    MeasurementUpdate update_;        ///< Generic measurement update

    double adaptive_noise_factor_;    ///< Scaling factor for adaptive noise
    double max_horizontal_error_;     ///< Maximum acceptable horizontal error [m]

    // Diagnostics
    Vector3d last_innovation_;        ///< Last innovation vector
    double last_adaptive_factor_;     ///< Last adaptive noise factor
    uint32_t consecutive_rejections_; ///< Track consecutive outliers

    /**
     * @brief Compute adaptive measurement noise covariance
     *
     * Uses Android's accuracy estimate as base noise.
     * Increases noise if accuracy is poor or consecutive rejections occur.
     *
     * @param location_meas Location measurement
     * @return Adaptive noise covariance R (3×3)
     */
    Matrix3d compute_adaptive_noise(const LocationMeasurement& location_meas) const;

    /**
     * @brief Compute measurement Jacobian
     *
     * H = ∂h/∂δx where h(x) = p_n (position)
     *
     * For position measurement (only depends on position error):
     * ∂h/∂δp = I_3x3 (identity matrix)
     * ∂h/∂(other states) = 0
     *
     * @return Jacobian H (3×15)
     */
    Eigen::Matrix<double, 3, 15> compute_jacobian() const;
};

}  // namespace fusion
```

---

### 2.4 Integration into Fusion Thread

**Modify**: `src/service/fusion_thread.hpp` (lines 340-352)

**Current**:
```cpp
// === Filter Components ===
EkfState ekf_state_;
ImuPreintegration imu_preint_;
MagUpdate mag_update_;

// === Sensor Queues ===
SPSCQueue<ImuSample, 512> imu_queue_;
SPSCQueue<MagSample, 64> mag_queue_;
```

**Add**:
```cpp
// === Filter Components ===
EkfState ekf_state_;
ImuPreintegration imu_preint_;
MagUpdate mag_update_;
LocationUpdate location_update_;  // NEW

// === Sensor Queues ===
SPSCQueue<ImuSample, 512> imu_queue_;
SPSCQueue<MagSample, 64> mag_queue_;
SPSCQueue<LocationMeasurement, 16> location_queue_;  // NEW
```

**Modify**: `src/service/fusion_thread.cpp`, `fusion_cycle()` function

**Add after magnetometer update block**:
```cpp
void FusionThread::fusion_cycle() {
    // ... (existing IMU preintegration and EKF prediction)

    // === STEP 3: Magnetometer Updates ===
    MagSample mag_sample;
    while (mag_queue_.pop(mag_sample)) {
        if (mag_update_.update(ekf_state_, mag_sample.mag_body)) {
            ekf_state_.inject_error(ekf_state_.error_state());
            ekf_state_.reset_error();
            stats_.mag_updates_processed++;
        }
    }

    // === STEP 4: Location Updates (NEW) ===
    LocationMeasurement location_meas;
    while (location_queue_.pop(location_meas)) {
        if (location_update_.update(ekf_state_, location_meas)) {
            // Update accepted
            ekf_state_.inject_error(ekf_state_.error_state());
            ekf_state_.reset_error();
            stats_.location_updates_processed++;
        } else {
            // Update rejected (outlier or poor accuracy)
            stats_.location_updates_rejected++;
        }
    }

    // ... (rest of fusion cycle)
}
```

**Add public method**:
```cpp
/**
 * @brief Push location measurement to queue
 * @return true if queued, false if overflow
 */
bool push_location_sample(const LocationMeasurement& location) {
    bool success = location_queue_.push(location);
    if (!success) {
        stats_.location_queue_overflows++;
    }
    return success;
}
```

---

### 2.5 State Estimation Implications

**No Changes to State Vector**:
- Current: 15D error state `[δθ, δv, δp, δb_g, δb_a]`
- With phone location: 15D (unchanged!)
- Original GNSS plan: 17D (adds clock_bias, clock_drift)

**Advantage**: No state expansion needed. Clock states were only required for raw pseudorange fusion.

**Covariance Impact**:
- Location measurement directly observes position (δp)
- H matrix has identity in position block: `H = [0, 0, I_3x3, 0, 0]`
- Position covariance shrinks after update
- Velocity/attitude covariance unaffected (no direct observability)

---

## 3. Implementation Roadmap

### Phase 1: Infrastructure (1-2 days)
1. Add `LocationMeasurement` struct to `sensor_types.hpp`
2. Create `jni_location.hpp/cpp` with JNI bridge
3. Implement WGS84→NED conversion (tangent plane approximation)
4. Add lock-free queue to `FusionThread`

### Phase 2: Measurement Update (1 day)
1. Create `location_update.hpp/cpp` following `mag_update` pattern
2. Implement `compute_jacobian()` (trivial: identity matrix)
3. Implement `compute_adaptive_noise()` (scale by accuracy)
4. Add `is_location_valid()` quality check

### Phase 3: Integration (1 day)
1. Integrate `LocationUpdate` into `fusion_cycle()`
2. Add `push_location_sample()` API
3. Update `ThreadStats` with location statistics
4. Add JNI exports for location callbacks

### Phase 4: Testing (2 days)
1. Unit tests: `test_location_update.cpp`
2. Integration test: `test_location_fusion.cpp`
3. Validation: Compare with ground truth trajectory
4. Performance: Benchmark update latency (<10 µs)

### Phase 5: Java/Kotlin Integration (1-2 days)
1. Create `LocationService.java` wrapper
2. Implement `FusedLocationProviderClient` integration
3. Add permission handling (ACCESS_FINE_LOCATION)
4. Test on real Android device

**Total Effort**: ~6-8 days (vs. 15-20 days for raw GNSS)

---

## 4. Performance Analysis

### Computational Cost

**Raw GNSS (planned)**:
- Iterate over 8-32 satellites
- Pseudorange prediction: ~2 µs × N_sats = 16-64 µs
- Jacobian computation: ~5 µs × N_sats = 40-160 µs
- Kalman update: ~20 µs (larger measurement dimension)
- **Total: ~76-244 µs per epoch**

**Phone Location (proposed)**:
- Single 3D position measurement
- Position innovation: ~0.5 µs
- Jacobian computation: trivial (identity matrix)
- Kalman update: ~5 µs (3D measurement)
- **Total: ~5-10 µs per update**

**Speedup**: 10-40× faster than raw GNSS!

### Memory Footprint

**Raw GNSS**:
- `GnssMeasurement`: ~3 KB per sample
- Queue capacity 16: 48 KB
- `GnssUpdate` class: ~1 KB (satellite geometry tables)
- **Total: ~49 KB**

**Phone Location**:
- `LocationMeasurement`: 64 bytes per sample
- Queue capacity 16: 1 KB
- `LocationUpdate` class: ~200 bytes
- **Total: ~1.2 KB**

**Memory saving**: 97.5% reduction!

### Update Rate

**Raw GNSS**: 1-10 Hz (typical: 1 Hz in Android)
**Phone Location**: 0.5-5 Hz (configurable via priority)

**Practical Impact**: Similar update rates, but location can go faster in HIGH_ACCURACY mode (up to 5 Hz).

---

## 5. Robustness Comparison

### Indoor/Urban Performance

| Scenario | Raw GNSS | Phone Location |
|----------|----------|----------------|
| **Open sky** | ⭐⭐⭐⭐⭐ 2-5m | ⭐⭐⭐⭐ 5-10m |
| **Urban canyon** | ⭐⭐ 10-50m (multipath) | ⭐⭐⭐⭐ 10-20m (WiFi fallback) |
| **Indoor** | ❌ No fix | ⭐⭐⭐ 5-20m (WiFi/BLE) |
| **Tunnel** | ❌ No fix | ⭐⭐ 50-500m (cell towers) |
| **Parking garage** | ❌ No fix | ⭐⭐⭐ 10-30m (WiFi) |

**Conclusion**: Phone location provides **graceful degradation** across all scenarios.

### Multi-Source Fusion

**Raw GNSS**: Single modality (satellite-only)
**Phone Location**: Multi-modal fusion by Android
- GPS L1/L5 (dual-frequency if available)
- WiFi positioning (Google/Skyhook databases)
- Cell tower triangulation (LTE/5G)
- Bluetooth beacons (indoor positioning)
- Dead reckoning (if motion detected)

**Advantage**: Android automatically switches sources based on availability and quality.

---

## 6. Trade-off Analysis

### What We Lose

1. **Observability**
   - Cannot weight individual satellites by CN0
   - Cannot perform elevation masking
   - Cannot detect and reject multipath on specific satellites
   - **Impact**: ~2-5m worse accuracy in challenging GNSS environments

2. **Latency**
   - Android FusedLocationProvider has ~100-300ms internal latency
   - Raw GNSS would have ~50-100ms latency
   - **Impact**: Slightly lagged position corrections (but IMU bridges gap)

3. **Clock States**
   - Cannot estimate receiver clock bias/drift
   - **Impact**: None (these states only useful for raw pseudorange fusion)

4. **Algorithmic Control**
   - Cannot implement custom outlier rejection beyond Android's
   - Cannot tune GNSS-specific parameters
   - **Impact**: Less control over fusion behavior

### What We Gain

1. **Simplicity**
   - 80% less code (~200 lines vs. ~800 lines)
   - No satellite ephemeris handling
   - No pseudorange modeling
   - No atmospheric corrections
   - **Impact**: Faster development, fewer bugs, easier maintenance

2. **Multi-Source Fusion**
   - Automatic WiFi/cell fallback
   - Indoor positioning capability
   - Proven robustness (billions of devices)
   - **Impact**: Works in more environments than GPS-only

3. **Battery Efficiency**
   - Android optimizes sensor usage across all apps
   - Can request lower priority updates when battery-constrained
   - **Impact**: Better battery life than raw GNSS

4. **Maintenance**
   - Android handles GNSS almanac/ephemeris updates
   - Automatic map corrections (Google Maps integration)
   - Benefits from continuous Android improvements
   - **Impact**: Free performance upgrades over time

---

## 7. Migration Path

### Option A: Full Replacement (Recommended)

**Action**: Replace all GNSS infrastructure with phone location
**Effort**: 6-8 days
**Risk**: Low (Android location is battle-tested)
**Recommendation**: ✅ **Do this**

**Steps**:
1. Remove `GnssMeasurement` struct (or deprecate)
2. Remove `jni_gnss.hpp` (or keep as stub for future)
3. Add `LocationMeasurement` and `jni_location.hpp`
4. Implement `LocationUpdate` class
5. Integrate into `FusionThread`

### Option B: Hybrid Approach

**Action**: Support both raw GNSS and phone location
**Effort**: 15-20 days (implement both paths)
**Risk**: Medium (more complexity, more testing)
**Recommendation**: ⚠️ **Not recommended** (adds complexity without clear benefit)

**Use Case**: Research/benchmarking (compare raw GNSS vs. fused location)

### Option C: Phased Approach

**Action**: Start with phone location, add raw GNSS later if needed
**Effort**: 6-8 days now, +15-20 days later if GNSS needed
**Risk**: Low (can always add raw GNSS later)
**Recommendation**: ✅ **Pragmatic approach**

**Rationale**: Ship v1.5 with phone location. If accuracy is insufficient in specific scenarios, add raw GNSS as "expert mode" later.

---

## 8. Accuracy Validation Plan

### Benchmark Scenarios

1. **Open Sky (Walking)**
   - Ground truth: RTK GPS (2-5cm accuracy)
   - Compare: Raw GNSS (if available) vs. Phone location vs. IMU-only
   - Metrics: Position error RMS, max error, convergence time

2. **Urban Canyon (Driving)**
   - Ground truth: High-quality GNSS/IMU (post-processed)
   - Compare: Phone location vs. IMU-only
   - Metrics: Track GPS dropout periods, WiFi fallback effectiveness

3. **Indoor (Walking)**
   - Ground truth: Motion capture or laser tracker
   - Compare: Phone location (WiFi/BLE) vs. IMU-only
   - Metrics: Position drift rate, loop closure error

4. **Mixed Environment**
   - Outdoor → Indoor → Outdoor transition
   - Measure: Seamless handoff between GPS and WiFi
   - Metrics: Position discontinuities at transitions

### Success Criteria

| Scenario | Target Accuracy (95%) | Max Latency | Update Rate |
|----------|----------------------|-------------|-------------|
| Open sky | <10m | <500ms | 1-5 Hz |
| Urban | <20m | <500ms | 1-5 Hz |
| Indoor | <30m | <1000ms | 0.5-2 Hz |

**Fallback**: If phone location doesn't meet targets, can implement raw GNSS as Option C.

---

## 9. Recommendation

**Adopt Phone Location Integration (Option A: Full Replacement)**

**Justification**:
1. **80% code reduction**: Simpler system, fewer bugs
2. **Broader applicability**: Works indoor + outdoor
3. **Faster time to market**: 6-8 days vs. 15-20 days
4. **Battle-tested**: Leverages Android's proven infrastructure
5. **Future-proof**: Can add raw GNSS later if needed (Option C)

**Trade-off Acceptance**:
- Accept ~2-5m worse accuracy in open sky (5-10m vs. 2-5m)
- Accept ~100-300ms additional latency (mitigated by IMU preintegration)
- Gain indoor positioning capability
- Gain automatic multi-source fusion

**This aligns with the DESIGN.md philosophy**: "Use the resources we have. Don't reinvent what Android already does well."

---

## 10. Next Steps

If proposal accepted:

1. **Architecture Review** (1 hour)
   - Present to team
   - Discuss trade-offs
   - Get sign-off

2. **Design Refinement** (2 hours)
   - Finalize data structures
   - Review WGS84→NED conversion accuracy requirements
   - Decide on adaptive noise strategy

3. **Implementation** (6-8 days)
   - Follow Phase 1-5 roadmap above
   - Daily progress updates

4. **Validation** (2-3 days)
   - Run benchmark scenarios
   - Compare with ground truth
   - Document results

5. **Documentation** (1 day)
   - Update DESIGN.md
   - Update API documentation
   - Create integration guide for app developers

**Total Timeline**: ~2 weeks to production-ready location integration

---

## Appendix A: WGS84 to NED Conversion

For converting latitude/longitude/altitude to local NED (North-East-Down) frame:

```cpp
Vector3d JniLocation::wgs84_to_ned(double lat_deg, double lon_deg, double alt_m) const {
    if (!reference_set_) {
        // Cannot convert without reference point
        return Vector3d::Zero();
    }

    // WGS84 ellipsoid constants
    constexpr double a = 6378137.0;           // Semi-major axis [m]
    constexpr double f = 1.0 / 298.257223563; // Flattening
    constexpr double e2 = 2*f - f*f;          // Eccentricity squared

    // Convert to radians
    const double lat_rad = lat_deg * M_PI / 180.0;
    const double lon_rad = lon_deg * M_PI / 180.0;
    const double ref_lat_rad = ref_lat_deg_ * M_PI / 180.0;
    const double ref_lon_rad = ref_lon_deg_ * M_PI / 180.0;

    // Radius of curvature in the prime vertical
    const double N = a / std::sqrt(1.0 - e2 * std::sin(ref_lat_rad) * std::sin(ref_lat_rad));

    // Small angle approximation (valid for <10km radius)
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
```

**Accuracy**: <1cm error for distances <10km from reference point.

---

## Appendix B: Java Integration Example

```java
// LocationService.java
package com.fusion;

import android.content.Context;
import android.location.Location;
import android.os.Looper;
import com.google.android.gms.location.*;

public class LocationService {
    private FusedLocationProviderClient fusedLocationClient;
    private LocationCallback locationCallback;

    static {
        System.loadLibrary("fusion_jni");
    }

    public LocationService(Context context) {
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(context);
        nativeInit();
    }

    public void startLocationUpdates(Priority priority, long intervalMs) {
        LocationRequest request = new LocationRequest.Builder(priority, intervalMs)
            .setMinUpdateIntervalMillis(intervalMs / 2)
            .setMaxUpdateDelayMillis(intervalMs * 2)
            .build();

        locationCallback = new LocationCallback() {
            @Override
            public void onLocationResult(LocationResult result) {
                Location location = result.getLastLocation();
                if (location != null) {
                    nativeOnLocation(
                        location.getElapsedRealtimeNanos(),
                        location.getLatitude(),
                        location.getLongitude(),
                        location.getAltitude(),
                        location.getAccuracy(),
                        location.hasVerticalAccuracy() ? location.getVerticalAccuracyMeters() : 999.0f,
                        location.getSpeed(),
                        location.hasSpeedAccuracy() ? location.getSpeedAccuracyMetersPerSecond() : 999.0f,
                        getProviderFlags(location)
                    );
                }
            }
        };

        fusedLocationClient.requestLocationUpdates(request, locationCallback, Looper.getMainLooper());
        nativeStart(priority.ordinal(), intervalMs / 1000.0);
    }

    private int getProviderFlags(Location location) {
        int flags = 0;
        String provider = location.getProvider();
        if ("gps".equals(provider)) flags |= 1;
        if ("network".equals(provider)) flags |= 2;  // WiFi
        // Could add BLE (8) and cell (4) flags if detectable
        return flags;
    }

    public void stopLocationUpdates() {
        if (locationCallback != null) {
            fusedLocationClient.removeLocationUpdates(locationCallback);
            nativeStop();
        }
    }

    // Native methods
    private native void nativeInit();
    private native boolean nativeStart(int priority, double intervalSec);
    private native void nativeStop();
    private native void nativeOnLocation(
        long timestampNs, double lat, double lon, double alt,
        float hAcc, float vAcc, float speed, float speedAcc, int flags);
}
```

---

**End of Proposal**
