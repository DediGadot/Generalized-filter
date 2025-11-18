# Phone Location Integration - Executive Summary

**TL;DR**: Replace planned raw GNSS integration with Android FusedLocationProvider. 80% less code, works indoors, 2 weeks faster to ship.

---

## Key Changes at a Glance

| Aspect | Current Plan (Raw GNSS) | Proposed (Phone Location) |
|--------|-------------------------|---------------------------|
| **Data Source** | Raw satellite pseudoranges | Pre-computed position (GPS+WiFi+Cell) |
| **Code Complexity** | ~800 lines | ~200 lines |
| **Implementation Time** | 15-20 days | 6-8 days |
| **State Dimension** | 17D (adds clock bias/drift) | 15D (unchanged) |
| **Memory per Sample** | 3 KB | 64 bytes |
| **Update Cost** | 76-244 µs | 5-10 µs |
| **Open Sky Accuracy** | 2-5m | 5-10m |
| **Indoor Capability** | ❌ No | ✅ Yes (5-30m) |
| **Urban Performance** | ⚠️ Multipath issues | ✅ WiFi fallback |

---

## Architecture Comparison

### Raw GNSS (v1.5 Plan)
```
Android LocationManager (registerGnssMeasurementsCallback)
    ↓
Raw satellite data: 8-32 pseudoranges per epoch
    ↓
Per-satellite processing:
  - Pseudorange prediction
  - Atmospheric corrections
  - Elevation masking
  - CN0 weighting
    ↓
Kalman update (8-32D measurement)
    ↓
Result: 2-5m accuracy (open sky only)
```

**Complexity**: Satellite ephemeris, multipath detection, clock bias estimation

### Phone Location (Proposed)
```
Android FusedLocationProvider (requestLocationUpdates)
    ↓
Pre-computed position: lat/lon/alt + accuracy
    ↓
Simple processing:
  - WGS84 → NED conversion
  - Accuracy-based noise scaling
  - Quality check
    ↓
Kalman update (3D measurement)
    ↓
Result: 5-30m accuracy (works everywhere)
```

**Simplicity**: Android handles all sensor fusion internally

---

## Technical Details (5-Minute Version)

### New Data Structure
```cpp
struct LocationMeasurement {
    double latitude_deg, longitude_deg, altitude_m;
    Vector3d position_ned;  // Converted to local frame
    float horizontal_accuracy_m;
    float vertical_accuracy_m;
    uint8_t provider_flags;  // GPS=1, WiFi=2, Cell=4, BLE=8
};
// Size: 64 bytes (vs. 3 KB for GnssMeasurement)
```

### New Update Class (follows MagUpdate pattern)
```cpp
class LocationUpdate {
    bool update(EkfState& state,
                const LocationMeasurement& location_meas);

    // Measurement model: z = p_n + v
    // Jacobian: H = [0, 0, I_3x3, 0, 0]  (identity on position)
    // Noise: R = diag(accuracy²) × adaptive_factor
};
```

### Integration Point
```cpp
void FusionThread::fusion_cycle() {
    // ... IMU preintegration, EKF prediction ...

    // Magnetometer updates
    MagSample mag;
    while (mag_queue_.pop(mag)) {
        mag_update_.update(ekf_state_, mag.mag_body);
    }

    // Location updates (NEW)
    LocationMeasurement loc;
    while (location_queue_.pop(loc)) {
        location_update_.update(ekf_state_, loc);
    }

    // ... state publish, health monitoring ...
}
```

---

## What We Gain

✅ **Simplicity**: 80% less code (~200 vs ~800 lines)

✅ **Indoor Positioning**: WiFi/BLE-based location when GPS unavailable

✅ **Urban Robustness**: Automatic fallback to WiFi in urban canyons

✅ **Faster Development**: 6-8 days vs. 15-20 days implementation

✅ **Battery Optimization**: Android manages sensor usage across apps

✅ **Proven Technology**: Billions of devices use FusedLocationProvider

✅ **No State Expansion**: Keep 15D state (vs. 17D for raw GNSS)

✅ **Performance**: 10-40× faster update (5-10µs vs. 76-244µs)

---

## What We Trade Off

⚠️ **Accuracy in Open Sky**: 5-10m vs. 2-5m (acceptable for AR glasses)

⚠️ **Latency**: +100-300ms Android internal processing (IMU bridges gap)

⚠️ **Control**: Cannot tune satellite-level parameters

⚠️ **Observability**: No direct access to individual satellite quality

---

## Real-World Performance

| Scenario | IMU-Only Drift | + Phone Location | + Raw GNSS (theoretical) |
|----------|----------------|------------------|-------------------------|
| **Outdoor walking** (5 min) | 50-100m | 5-10m | 2-5m |
| **Urban driving** (10 min) | 200-500m | 10-20m | 10-50m (multipath) |
| **Indoor shopping** (5 min) | 50-100m | 5-30m (WiFi) | ❌ No fix |
| **Mixed transition** | Large jump | Smooth | Large jump |

**Conclusion**: Phone location provides **consistent performance** across all scenarios.

---

## Implementation Roadmap

```
Week 1:
├─ Day 1-2: Infrastructure
│   ├─ Add LocationMeasurement struct
│   ├─ Create jni_location.hpp/cpp
│   └─ Implement WGS84→NED conversion
│
├─ Day 3: Measurement Update
│   ├─ Create location_update.hpp/cpp
│   ├─ Implement Jacobian (trivial)
│   └─ Implement adaptive noise
│
└─ Day 4: Integration
    ├─ Integrate into FusionThread
    ├─ Add JNI callbacks
    └─ Update statistics

Week 2:
├─ Day 5-6: Testing
│   ├─ Unit tests
│   ├─ Integration tests
│   └─ Performance validation
│
└─ Day 7-8: Java Integration
    ├─ Create LocationService.java
    ├─ Implement FusedLocationProvider
    └─ Test on real device
```

**Total**: 2 weeks to production

---

## Recommendation: ✅ ADOPT

**Rationale**:
1. Aligns with DESIGN.md philosophy: "Use the resources we have. Don't reinvent what Android already does well."
2. Faster time to market (2 weeks vs. 4 weeks)
3. Works in more environments (indoor + outdoor)
4. Simpler codebase (fewer bugs, easier maintenance)
5. Can always add raw GNSS later if accuracy is insufficient

**Risk**: Low. FusedLocationProvider is battle-tested by billions of devices.

**Fallback**: If accuracy doesn't meet requirements, implement raw GNSS as "expert mode" (Option C in full proposal).

---

## Files to Review

1. **Full Proposal**: `docs/PHONE_LOCATION_PROPOSAL.md` (this summary's parent document)
   - 10 sections covering all details
   - Complete code examples
   - Performance analysis
   - Migration path options

2. **Current Architecture**: `DESIGN.md`
   - See Decision 1-10 for context
   - Threading model (lines 1030-1095)
   - Performance budget

3. **Existing Infrastructure**:
   - `src/core/sensor_types.hpp` (lines 84-97): GnssMeasurement struct (to replace)
   - `src/sensors/jni_gnss.hpp`: JNI bridge (to replace with jni_location.hpp)
   - `src/filter/mag_update.hpp`: Pattern to follow for LocationUpdate

---

## Next Actions

**If accepted**:
1. Schedule architecture review meeting (1 hour)
2. Get team sign-off on trade-offs
3. Begin Phase 1 implementation (infrastructure)
4. Daily progress updates

**If rejected**:
1. Clarify concerns (accuracy? control? other?)
2. Consider Option C: Start with phone location, add raw GNSS later
3. Or proceed with original raw GNSS plan (15-20 days)

---

**Questions?** See full proposal in `docs/PHONE_LOCATION_PROPOSAL.md`
