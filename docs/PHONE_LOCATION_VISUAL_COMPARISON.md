# Phone Location vs Raw GNSS - Visual Comparison

This document provides side-by-side visual comparisons to make the architectural differences immediately clear.

---

## 1. Data Flow Comparison

### Raw GNSS (Tightly-Coupled)

```
┌──────────────────────────────────────────────────────────────┐
│                    Android Hardware                          │
│                  GPS/GLONASS/Galileo/BeiDou                  │
│                    L1/L5 Dual Frequency                      │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│            Android LocationManager API                       │
│    registerGnssMeasurementsCallback()                       │
│    • 8-32 satellites per epoch                               │
│    • Pseudorange: 20M-26M meters (±1-50m noise)             │
│    • Doppler: ±1000 m/s (±0.1 m/s noise)                    │
│    • CN0: 20-50 dB-Hz (signal quality)                      │
│    • Satellite positions: ECEF coordinates                   │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                    JNI Bridge                                │
│    Marshal Java arrays → C++ structs                         │
│    • prns[] (satellite IDs)                                  │
│    • pseudoranges[] (8-32 doubles)                          │
│    • dopplers[] (8-32 floats)                               │
│    • cn0s[] (8-32 floats)                                   │
│    • sat_positions[] (8-32 × 3D vectors)                    │
│    COST: ~5-10 µs per epoch                                 │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│           Lock-free Queue (Capacity: 16)                     │
│           Size: 3 KB per measurement                         │
│           Total memory: 48 KB                                │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                    Fusion Thread                             │
│                                                              │
│  GnssUpdate::update(state, gnss_meas) {                     │
│                                                              │
│    // Step 1: Convert EKF state to ECEF                     │
│    Vector3d p_ecef = ned_to_ecef(state.position());         │
│    // COST: ~2 µs                                           │
│                                                              │
│    // Step 2: For EACH satellite (8-32 iterations)          │
│    for (int i = 0; i < num_sats; i++) {                     │
│                                                              │
│        // Predict pseudorange                               │
│        double range = ||p_ecef - sat_pos[i]||;             │
│        double predicted = range + c * clock_bias;           │
│        // COST: ~2 µs × N_sats = 16-64 µs                  │
│                                                              │
│        // Compute Jacobian                                  │
│        Vector3d unit_vec = (p_ecef - sat_pos[i]) / range;  │
│        H.block<1,3>(i,6) = unit_vec.transpose();           │
│        H(i, 15) = c;  // clock bias derivative             │
│        // COST: ~5 µs × N_sats = 40-160 µs                 │
│                                                              │
│        // Weight by signal quality                          │
│        double weight = cn0_to_weight(cn0[i]);              │
│        R(i,i) = base_noise² / weight;                       │
│                                                              │
│        // Elevation masking                                 │
│        if (elevation[i] < 15°) continue;                    │
│                                                              │
│        // Atmospheric corrections                           │
│        apply_ionosphere_correction(predicted, elevation[i]);│
│        apply_troposphere_correction(predicted, elevation[i]);│
│        // COST: ~3 µs × N_sats = 24-96 µs                  │
│    }                                                        │
│                                                              │
│    // Step 3: Kalman update (8-32D measurement!)            │
│    innovation = measured - predicted;                       │
│    S = H * P * H.T + R;                                    │
│    K = P * H.T * S.inv();                                  │
│    dx = K * innovation;                                     │
│    P = (I - K*H) * P * (I - K*H).T + K*R*K.T;             │
│    // COST: ~20 µs (larger matrices)                       │
│                                                              │
│    // TOTAL: 76-244 µs per update                          │
│  }                                                          │
└──────────────────────────────────────────────────────────────┘
```

### Android Fused Location (Proposed)

```
┌──────────────────────────────────────────────────────────────┐
│               Android Multi-Source Fusion                    │
│   GPS + WiFi + Cell Towers + BLE + Dead Reckoning          │
│   (Handled internally by FusedLocationProvider)             │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│          FusedLocationProviderClient API                     │
│    requestLocationUpdates(priority, interval)                │
│    • Pre-computed position: lat/lon/alt                     │
│    • Accuracy estimates: horizontal/vertical                 │
│    • Provider info: GPS/WiFi/Cell flags                     │
│    • Simple Location object (8 fields)                      │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                    JNI Bridge                                │
│    Marshal Location object → C++ struct                      │
│    • latitude_deg (1 double)                                │
│    • longitude_deg (1 double)                               │
│    • altitude_m (1 double)                                  │
│    • horizontal_accuracy_m (1 float)                        │
│    • vertical_accuracy_m (1 float)                          │
│    COST: <1 µs per measurement                              │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│           Lock-free Queue (Capacity: 16)                     │
│           Size: 64 bytes per measurement                     │
│           Total memory: 1 KB                                 │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                    Fusion Thread                             │
│                                                              │
│  LocationUpdate::update(state, loc_meas) {                   │
│                                                              │
│    // Step 1: Convert WGS84 → NED (if not already)         │
│    Vector3d z_ned = wgs84_to_ned(                          │
│        loc_meas.latitude_deg,                               │
│        loc_meas.longitude_deg,                              │
│        loc_meas.altitude_m);                                │
│    // COST: ~1 µs (simple tangent plane)                   │
│                                                              │
│    // Step 2: Predict measurement (trivial)                 │
│    Vector3d predicted = state.position();                   │
│    // COST: ~0.1 µs                                         │
│                                                              │
│    // Step 3: Compute innovation                            │
│    Vector3d innovation = z_ned - predicted;                 │
│    // COST: ~0.1 µs                                         │
│                                                              │
│    // Step 4: Build Jacobian (trivial identity matrix)      │
│    // H = [0, 0, I_3x3, 0, 0] (3×15 matrix)                │
│    // Only position block is identity, rest is zero         │
│    // COST: ~0.5 µs (mostly zeros)                         │
│                                                              │
│    // Step 5: Adaptive noise                                │
│    double h_acc = loc_meas.horizontal_accuracy_m;          │
│    double v_acc = loc_meas.vertical_accuracy_m;            │
│    double factor = 1.0 + (h_acc > 50.0 ? 2.0 : 0.0);      │
│    R = diag(h_acc², h_acc², v_acc²) * factor²;             │
│    // COST: ~0.5 µs                                         │
│                                                              │
│    // Step 6: Kalman update (3D measurement)                │
│    S = H * P * H.T + R;                                    │
│    K = P * H.T * S.inv();                                  │
│    dx = K * innovation;                                     │
│    P = (I - K*H) * P * (I - K*H).T + K*R*K.T;             │
│    // COST: ~5 µs (smaller matrices)                       │
│                                                              │
│    // TOTAL: 5-10 µs per update                            │
│  }                                                          │
└──────────────────────────────────────────────────────────────┘
```

**Key Difference**: 8-32 iterations vs. single iteration

---

## 2. Code Complexity Comparison

### Raw GNSS Implementation

```
src/sensors/jni_gnss.hpp               (150 lines)
src/sensors/jni_gnss.cpp               (200 lines)
src/filter/gnss_update.hpp             (180 lines)
src/filter/gnss_update.cpp             (450 lines)
src/utils/coordinate_transforms.hpp    (100 lines)
src/utils/coordinate_transforms.cpp    (200 lines)
src/validation/test_gnss_update.cpp    (300 lines)
────────────────────────────────────────────────
TOTAL:                                 ~1580 lines

Components needed:
├─ Satellite ephemeris handling
├─ Pseudorange modeling
├─ Doppler shift computation
├─ Atmospheric corrections (ionosphere/troposphere)
├─ Elevation masking
├─ CN0-based weighting
├─ ECEF ↔ NED coordinate transforms
├─ Clock bias/drift state management (17D state)
├─ Per-satellite Jacobian computation
└─ Multi-satellite measurement construction
```

### Phone Location Implementation

```
src/sensors/jni_location.hpp           (120 lines)
src/sensors/jni_location.cpp           (150 lines)
src/filter/location_update.hpp         (120 lines)
src/filter/location_update.cpp         (180 lines)
src/validation/test_location_update.cpp (200 lines)
────────────────────────────────────────────────
TOTAL:                                 ~770 lines

Components needed:
├─ Simple Location object marshalling
├─ WGS84 → NED conversion (tangent plane)
├─ Accuracy-based adaptive noise
├─ Quality check (accuracy thresholds)
└─ Simple 3D Jacobian (mostly identity matrix)
```

**Code Reduction**: 810 lines (51% reduction)

---

## 3. State Dimension Comparison

### Raw GNSS: 17D State

```
Error State (17D):
┌────────────────────────────────────┐
│ δθ:  Rotation error       [0:3]   │  3D (axis-angle)
│ δv:  Velocity error       [3:6]   │  3D (North, East, Down)
│ δp:  Position error       [6:9]   │  3D (North, East, Down)
│ δbg: Gyro bias error      [9:12]  │  3D (body frame)
│ δba: Accel bias error     [12:15] │  3D (body frame)
│ δcb: Clock bias error     [15]    │  1D (meters)
│ δcd: Clock drift error    [16]    │  1D (meters/sec)
└────────────────────────────────────┘

Covariance Matrix: 17×17 = 289 elements
Memory: 289 × 8 bytes = 2.3 KB

WHY clock states?
- Raw pseudorange = geometric_range + c·clock_bias + noise
- Clock bias changes slowly (~meters per second drift)
- Clock drift = rate of clock bias change
- These states are ONLY needed for raw pseudorange fusion
```

### Phone Location: 15D State

```
Error State (15D):
┌────────────────────────────────────┐
│ δθ:  Rotation error       [0:3]   │  3D (axis-angle)
│ δv:  Velocity error       [3:6]   │  3D (North, East, Down)
│ δp:  Position error       [6:9]   │  3D (North, East, Down)
│ δbg: Gyro bias error      [9:12]  │  3D (body frame)
│ δba: Accel bias error     [12:15] │  3D (body frame)
└────────────────────────────────────┘

Covariance Matrix: 15×15 = 225 elements
Memory: 225 × 8 bytes = 1.8 KB

WHY no clock states?
- Android provides pre-computed position (already corrected)
- Clock bias is handled internally by Android
- Measurement model: z = position (direct observation)
- No need to model receiver clock in our filter
```

**Memory Saving**: 512 bytes per state

---

## 4. Measurement Jacobian Comparison

### Raw GNSS: Per-Satellite Jacobian

```
For satellite i:

Measurement: zi = ||p_ecef - sat_pos_i|| + c·clock_bias + noise

Jacobian Hi (1×17):
┌────────────────────────────────────────────────────────────────┐
│  ∂zi/∂θ  │  ∂zi/∂v  │  ∂zi/∂p  │ ∂zi/∂bg │ ∂zi/∂ba │ ∂zi/∂cb │ ∂zi/∂cd │
│          │          │          │         │         │         │         │
│    0     │    0     │ unit_veci│    0    │    0    │    c    │    0    │
│  (3×1)   │  (3×1)   │  (3×1)   │  (3×1)  │  (3×1)  │  (1×1)  │  (1×1)  │
└────────────────────────────────────────────────────────────────┘

where:
  unit_veci = (p_ecef - sat_pos_i) / ||p_ecef - sat_pos_i||
  c = speed of light (299,792,458 m/s)

For N satellites, stack N Jacobians:
H = [H1; H2; ...; HN]  (N×17 matrix)

Computation:
- Compute ECEF position from NED (rotation matrix)
- Compute unit vector for each satellite
- Build sparse matrix (mostly zeros)
COST: ~5 µs × N = 40-160 µs
```

### Phone Location: Simple Jacobian

```
Measurement: z = [north, east, down] (3D position)

Jacobian H (3×15):
┌────────────────────────────────────────────────────────────────┐
│  ∂z/∂θ  │  ∂z/∂v  │  ∂z/∂p  │ ∂z/∂bg │ ∂z/∂ba │
│          │         │         │        │        │
│    0     │    0    │   I3×3  │   0    │   0    │
│  (3×3)   │  (3×3)  │  (3×3)  │ (3×3)  │ (3×3)  │
└────────────────────────────────────────────────────────────────┘

where:
  I3×3 = identity matrix (ones on diagonal, zeros elsewhere)
  0 = zero matrix

Simplified (only position block is non-zero):
H = [0₃ₓ₃, 0₃ₓ₃, I₃ₓ₃, 0₃ₓ₃, 0₃ₓ₃]

Computation:
- Hardcode identity matrix for position block
- Rest is zeros
COST: ~0.5 µs (trivial)
```

**Speedup**: 80-320× faster Jacobian computation!

---

## 5. Accuracy vs Environment Comparison

```
Performance Map:

Open Sky (Parks, Highways):
├─ Raw GNSS:       ████████████████████ 2-5m (excellent)
├─ Phone Location: ████████████         5-10m (good)
└─ IMU-only:       ██                   50-100m (poor, drifts)

Suburban (Trees, Buildings):
├─ Raw GNSS:       ████████████         5-15m (good)
├─ Phone Location: ████████████         5-15m (good, WiFi helps)
└─ IMU-only:       ██                   50-100m (poor)

Urban Canyon (Downtown):
├─ Raw GNSS:       ████                 10-50m (poor, multipath)
├─ Phone Location: ████████████         10-20m (good, WiFi fallback)
└─ IMU-only:       ██                   100-200m (poor)

Indoor (Malls, Buildings):
├─ Raw GNSS:       ×                    No fix
├─ Phone Location: ████████             5-30m (WiFi/BLE)
└─ IMU-only:       ██                   50-100m (poor)

Underground (Tunnels, Parking):
├─ Raw GNSS:       ×                    No fix
├─ Phone Location: ████                 50-500m (cell towers)
└─ IMU-only:       ██                   100-200m (poor)

Legend: █ = 5m accuracy
```

**Conclusion**: Phone location provides **consistent coverage** across all environments.

---

## 6. Implementation Timeline Comparison

### Raw GNSS Timeline (4 Weeks)

```
Week 1: Core Infrastructure
├─ Day 1-2: GnssMeasurement struct, JNI bridge
├─ Day 3-4: Coordinate transforms (ECEF ↔ NED ↔ WGS84)
└─ Day 5:   Lock-free queue integration

Week 2: Measurement Update
├─ Day 6-7: Pseudorange prediction model
├─ Day 8:   Doppler shift model
├─ Day 9:   Atmospheric corrections
└─ Day 10:  Elevation masking, CN0 weighting

Week 3: State Extension & Jacobian
├─ Day 11-12: Extend state to 17D (clock bias/drift)
├─ Day 13-14: Per-satellite Jacobian computation
└─ Day 15:    Multi-satellite measurement construction

Week 4: Testing & Integration
├─ Day 16-17: Unit tests, integration tests
├─ Day 18:    Java integration (LocationManager)
├─ Day 19:    Validation with ground truth
└─ Day 20:    Bug fixes, optimization

TOTAL: 20 days (4 weeks)
```

### Phone Location Timeline (2 Weeks)

```
Week 1: Implementation
├─ Day 1:   LocationMeasurement struct, JNI bridge
├─ Day 2:   WGS84 → NED conversion (tangent plane)
├─ Day 3:   LocationUpdate class (follow MagUpdate pattern)
├─ Day 4:   Integration into FusionThread
└─ Day 5:   Unit tests, performance validation

Week 2: Integration & Testing
├─ Day 6:   Java integration (FusedLocationProvider)
├─ Day 7:   Integration tests, end-to-end validation
└─ Day 8:   Documentation, final polish

TOTAL: 8 days (2 weeks)
```

**Time Saved**: 12 days (60% faster)

---

## 7. Memory Usage Comparison

### Per-Measurement Memory

```
Raw GNSS Measurement:
┌─────────────────────────────────────────┐
│ GnssMeasurement struct:                 │
│ ├─ timestamp_ns         8 bytes         │
│ ├─ num_sats             1 byte          │
│ ├─ padding              7 bytes         │
│ ├─ clock_bias           8 bytes         │
│ ├─ clock_drift          8 bytes         │
│ └─ sats[32]:                            │
│     ├─ prn                1 byte × 32   │
│     ├─ constellation      1 byte × 32   │
│     ├─ padding            6 bytes × 32  │
│     ├─ pseudorange        8 bytes × 32  │
│     ├─ doppler            4 bytes × 32  │
│     ├─ cn0                4 bytes × 32  │
│     ├─ elevation          4 bytes × 32  │
│     ├─ azimuth            4 bytes × 32  │
│     ├─ sat_pos_ecef      24 bytes × 32  │
│     └─ sat_vel_ecef      24 bytes × 32  │
│         = 80 bytes × 32 = 2560 bytes    │
└─────────────────────────────────────────┘
TOTAL: ~2600 bytes per measurement

Queue (capacity 16): 2600 × 16 = 41,600 bytes ≈ 41 KB
```

```
Phone Location Measurement:
┌─────────────────────────────────────────┐
│ LocationMeasurement struct:             │
│ ├─ timestamp_ns           8 bytes       │
│ ├─ latitude_deg           8 bytes       │
│ ├─ longitude_deg          8 bytes       │
│ ├─ altitude_m             8 bytes       │
│ ├─ position_ned          24 bytes       │
│ ├─ horizontal_accuracy_m  4 bytes       │
│ ├─ vertical_accuracy_m    4 bytes       │
│ ├─ speed_accuracy_mps     4 bytes       │
│ ├─ provider_flags         1 byte        │
│ ├─ num_satellites         1 byte        │
│ └─ padding                2 bytes       │
└─────────────────────────────────────────┘
TOTAL: 64 bytes per measurement

Queue (capacity 16): 64 × 16 = 1,024 bytes ≈ 1 KB
```

**Memory Saved per Queue**: 40 KB (97.5% reduction)

---

## 8. Final Recommendation

### Decision Matrix

| Criterion | Raw GNSS | Phone Location | Winner |
|-----------|----------|----------------|--------|
| **Open Sky Accuracy** | ⭐⭐⭐⭐⭐ (2-5m) | ⭐⭐⭐⭐ (5-10m) | Raw GNSS |
| **Urban Accuracy** | ⭐⭐ (10-50m) | ⭐⭐⭐⭐ (10-20m) | Phone Location |
| **Indoor Capability** | ❌ No fix | ⭐⭐⭐ (5-30m) | Phone Location |
| **Implementation Time** | ⭐⭐ (4 weeks) | ⭐⭐⭐⭐⭐ (2 weeks) | Phone Location |
| **Code Complexity** | ⭐⭐ (1580 lines) | ⭐⭐⭐⭐⭐ (770 lines) | Phone Location |
| **Computational Cost** | ⭐⭐ (76-244 µs) | ⭐⭐⭐⭐⭐ (5-10 µs) | Phone Location |
| **Memory Usage** | ⭐⭐ (41 KB queue) | ⭐⭐⭐⭐⭐ (1 KB queue) | Phone Location |
| **Battery Efficiency** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | Phone Location |
| **Maintainability** | ⭐⭐ | ⭐⭐⭐⭐⭐ | Phone Location |

**Score**: Phone Location wins 8 out of 9 categories

### Recommendation: ✅ **ADOPT PHONE LOCATION**

**Rationale**:
1. **Pragmatic**: Leverages Android's proven infrastructure
2. **Simpler**: 51% less code, 97% less memory
3. **Faster**: 60% faster to implement, 10-40× faster execution
4. **Robust**: Works in more environments (indoor + outdoor)
5. **Future-proof**: Can add raw GNSS later if needed

**Trade-off**: Accept ~5m worse accuracy in ideal conditions for consistent performance everywhere.

---

## Next Steps

1. **Review**: Present to team, discuss trade-offs
2. **Decide**: Get sign-off on approach
3. **Implement**: Follow 2-week roadmap
4. **Validate**: Test in real-world scenarios
5. **Ship**: Deploy v1.5 with phone location

**Fallback**: If accuracy is insufficient, add raw GNSS as "expert mode" in v2.0.

---

**Documents**:
- **This document**: Visual comparison
- **Summary**: `docs/PHONE_LOCATION_SUMMARY.md`
- **Full proposal**: `docs/PHONE_LOCATION_PROPOSAL.md`
