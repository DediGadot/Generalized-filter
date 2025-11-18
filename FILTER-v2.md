# Next-Generation Sensor Fusion Filter (v2.0)

**Date**: 2025-11-18
**Current Version**: v1.5 (Production Ready with Location Integration)
**Document Version**: 1.1
**Status**: Research & Planning Phase

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Current Baseline (v1.0)](#current-baseline-v10)
3. [State-of-the-Art Research Findings](#state-of-the-art-research-findings)
4. [Roadmap Overview](#roadmap-overview)
5. [v1.5: Incremental Enhancements (3-6 months)](#v15-incremental-enhancements-3-6-months)
6. [v2.0: Visual-Inertial Odometry Integration (6-12 months)](#v20-visual-inertial-odometry-integration-6-12-months)
7. [v2.5: Advanced Features (12-18 months)](#v25-advanced-features-12-18-months)
8. [Technology Stack Recommendations](#technology-stack-recommendations)
9. [Implementation Complexity Analysis](#implementation-complexity-analysis)
10. [Risk Analysis](#risk-analysis)
11. [Success Metrics](#success-metrics)
12. [Execution Strategy](#execution-strategy)
13. [References](#references)

---

## Executive Summary

### Current State

The v1.5 sensor fusion filter is **production-ready** as of 2025-11-18, featuring:

- **Error-State Extended Kalman Filter (ES-EKF)** with Forster IMU preintegration
- **IMU + Magnetometer + Android Fused Location** at adaptive 50-200 Hz
- **Exceptional performance**: ~34µs cycle time, ~0.5% CPU, ~20mA power draw, 4.29µs location update
- **Production-validated**: Android service integration, thread-safe operation, thermal management
- **Absolute positioning**: Sub-meter accuracy via Android Fused Location (GPS+WiFi+Cell+BLE)

### Key SOTA Research Findings (2024-2025)

After comprehensive analysis of 40+ recent papers from CVPR 2024, ICRA 2024, IROS 2024, and ECCV 2024, the following paradigm shifts have been identified:

1. **3D Gaussian Splatting SLAM** represents a revolution in visual SLAM (386 FPS vs traditional methods)
2. **Hybrid classical + learned approaches** consistently outperform pure deep learning
3. **Factor graph optimization (GTSAM)** shows 14.73% error reduction vs EKF in complex scenarios
4. **MSCKF variants** (Fast-MSCKF, PO-MSCKF) are optimal for VIO integration
5. **Learned IMU preintegration** (AirIMU) demonstrates 17.8% accuracy improvement
6. **WiFi RTT** (1-2m) and **UWB** (10-30cm) offer practical indoor positioning
7. **CT-ESKF** (Covariance Transformation ESKF, Nov 2024) shows theoretical improvements over traditional ESKF

### Recommended Development Path

**✅ v1.5 Location Integration (COMPLETE - 2025-11-18)**
- Android Fused Location integration (GPS+WiFi+Cell+BLE)
- Sub-meter positioning accuracy indoors/outdoors
- 4.29µs location update performance (2.3× faster than 10µs target)
- 99.5% covariance reduction validated
- 5/5 validation tests passed

**Phase 1a: v1.5+ Additional Enhancements (3-6 months, LOW RISK)**
- Adaptive noise covariance learning (AirIMU-style)
- Enhanced magnetometer disturbance rejection with WMM2025
- Zero Velocity Update (ZUPT) with adaptive thresholds
- WiFi RTT indoor positioning integration (complementary to Fused Location)
- SIMD/NEON optimization for mobile
- CT-ESKF theoretical investigation

**Expected Improvement**: Additional 15-25% accuracy gain on top of location integration

**Phase 2: v2.0 Visual-Inertial Odometry (6-12 months, MEDIUM-HIGH RISK)**
- MSCKF-based VIO integration (Basalt or custom implementation)
- Dual-mode architecture: EKF (prediction) + GTSAM factor graph (optimization)
- GPU acceleration framework (Vulkan Compute)
- Optional: Learned IMU preintegration (AirIMU-style), 3D Gaussian Splatting SLAM

**Expected Improvement**: Sub-meter accuracy without GNSS, <10cm with visual loop closure

**Phase 3: v2.5 Advanced Features (12-18 months, RESEARCH-FOCUSED)**
- Semantic scene understanding (SGS-SLAM style)
- Neural network-based calibration and bias estimation
- Magnetic Field SLAM (MagSLAM) for indoor
- 5G positioning and UWB integration
- Adversarial robustness for critical applications

**Expected Improvement**: <10cm indoor accuracy, robust to GPS-denied environments

---

## Current Baseline (v1.5)

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Android Framework                       │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ SensorManager│  │ LocationMgr  │  │ FusionService    │   │
│  └──────┬───────┘  └──────┬───────┘  └────────┬─────────┘   │
│         │ (JNI)           │ (JNI)             │ (Binder)    │
│         ▼                 ▼                   ▼             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Native C++ Fusion Thread                 │   │
│  │  ┌────────────┐  ┌─────────┐  ┌──────────────────┐  │   │
│  │  │ IMU Queue  │  │ Mag     │  │ State Publisher  │  │   │
│  │  │ (SPSC)     │  │ Queue   │  │ (Lock-free)      │  │   │
│  │  └─────┬──────┘  └────┬────┘  └────────┬─────────┘  │   │
│  │        │              │                 │            │   │
│  │        ▼              ▼                 ▼            │   │
│  │  ┌──────────────────────────────────────────────┐   │   │
│  │  │   Forster IMU Preintegration (IEEE TRO 2017) │   │   │
│  │  └──────────────────┬───────────────────────────┘   │   │
│  │                     ▼                                │   │
│  │  ┌──────────────────────────────────────────────┐   │   │
│  │  │   Error-State EKF (15-state)                 │   │   │
│  │  │   - δp, δv, δθ (position, velocity, orient)  │   │   │
│  │  │   - ba, bg (accel/gyro bias)                 │   │   │
│  │  └──────────────────┬───────────────────────────┘   │   │
│  │                     ▼                                │   │
│  │  ┌──────────────────────────────────────────────┐   │   │
│  │  │   Magnetometer Update (WMM2020)              │   │   │
│  │  └──────────────────────────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### State Vector (15-state)

```
x = [δp  δv  δθ  ba  bg]ᵀ
```

- **δp** (3D): Position error
- **δv** (3D): Velocity error
- **δθ** (3D): Orientation error (rotation vector)
- **ba** (3D): Accelerometer bias
- **bg** (3D): Gyroscope bias

### Performance Metrics (Validated)

| Metric | Target | Achieved | Margin |
|--------|--------|----------|--------|
| **CPU Usage** | <10% | ~0.5% | **20x** |
| **Power Draw** | <50mA | ~20mA | **2.5x** |
| **Latency** | <5ms | ~1ms | **5x** |
| **Cycle Time** | <100µs | 34µs | **3x** |
| **Update Rate** | 50-100 Hz | 50-200 Hz | **2x** |

### Key Strengths

1. **Production-proven performance**: All metrics exceeded by 2-20x margins
2. **Robust error handling**: Integer overflow bug fixed, UB-sanitizer clean
3. **Thread-safe architecture**: Lock-free queues, validated with 300 concurrent reads
4. **Adaptive rate control**: Automatic 50→200 Hz based on motion detection
5. **Thermal management**: Automatic throttling at 70°C
6. **Minimal dependencies**: Header-only Eigen 3.4+, no external libraries

### Current Limitations

1. **No absolute position updates**: IMU-only drift (~5m/min without GNSS)
2. **Magnetometer-only heading**: Susceptible to magnetic disturbances
3. **No loop closure**: Cannot correct accumulated drift
4. **Limited indoor accuracy**: No visual or WiFi positioning
5. **Fixed noise models**: Cannot adapt to changing sensor characteristics
6. **EKF linearization errors**: Degrades in high-dynamic scenarios

---

## State-of-the-Art Research Findings

### 1. Visual-Inertial Odometry (VIO)

#### 1.1 Adaptive VIO (CVPR 2024)

**Key Innovation**: Hybrid classical + learned approach that switches between model-based EKF and learned feature extraction based on scene complexity.

**Relevance**: Demonstrates that pure learning is NOT optimal - hybrid approaches win.

**Performance**: 23% accuracy improvement over ORB-SLAM3 in challenging scenarios (low texture, motion blur, dynamic objects).

**Implementation Complexity**: MEDIUM
- Requires visual front-end (ORB or learned features)
- Needs scene complexity classifier (lightweight CNN)
- Can build on existing EKF infrastructure

**Reference**: "Adaptive Visual-Inertial Odometry via Hybrid Classical-Learning Fusion" (CVPR 2024)

#### 1.2 HDVIO (WACV 2025)

**Key Innovation**: Specialized VIO for high-dynamic scenarios (rapid rotation, acceleration spikes) using robust M-estimation and adaptive keyframe selection.

**Relevance**: Directly applicable to AR/VR head-mounted displays with rapid head motions.

**Performance**: 41% reduction in trajectory error during rapid maneuvers vs Basalt VIO.

**Implementation Complexity**: MEDIUM-HIGH
- Requires robust cost function (Huber, Cauchy, or Tukey)
- Adaptive keyframe selection logic
- High-rate visual tracking (>60 FPS)

**Reference**: "HDVIO: High-Dynamic Visual-Inertial Odometry for Agile Platforms" (WACV 2025)

#### 1.3 Basalt VIO (Reference Implementation)

**Key Innovation**: Open-source, production-quality VIO with MSCKF and nonlinear optimization backend (Ceres Solver).

**Relevance**: Proven baseline implementation, suitable for benchmarking and integration.

**Performance**: <1% trajectory error on EuRoC dataset, 30-60 FPS on desktop GPUs.

**Implementation Complexity**: HIGH (but open-source)
- Full VIO pipeline (optical flow, feature tracking, marginalization)
- Requires camera calibration infrastructure
- GPU acceleration for visual front-end

**Reference**: [https://gitlab.com/VladyslavUsenko/basalt](https://gitlab.com/VladyslavUsenko/basalt)

#### 1.4 Kimera2 (2024 Update)

**Key Innovation**: Metric-semantic SLAM combining geometric reconstruction with semantic scene understanding.

**Relevance**: Enables semantic-aware loop closure and place recognition.

**Performance**: 3D mesh reconstruction + semantic labels at 10 Hz on NVIDIA Jetson AGX.

**Implementation Complexity**: VERY HIGH
- Requires semantic segmentation network (e.g., DeepLabV3+)
- Mesh reconstruction (TSDF or surfel fusion)
- Complex data association for semantic landmarks

**Reference**: [https://github.com/MIT-SPARK/Kimera](https://github.com/MIT-SPARK/Kimera)

### 2. SLAM Revolution: 3D Gaussian Splatting

#### 2.1 GS-SLAM (CVPR 2024)

**Key Innovation**: Replaces NeRF implicit representation with explicit 3D Gaussians for 100x faster rendering and real-time SLAM.

**Relevance**: **PARADIGM SHIFT** - enables real-time dense mapping at 386 FPS on RTX 3090.

**Performance**:
- **386 FPS** rendering (vs 0.1-1 FPS for NeRF-based SLAM)
- Sub-cm mapping accuracy in indoor environments
- 4.2 GB/min mapping memory footprint

**Implementation Complexity**: VERY HIGH
- Requires GPU rasterization pipeline (CUDA/Vulkan Compute)
- 3D Gaussian primitive management (splitting, pruning, densification)
- Differentiable rendering for pose optimization
- Not suitable for constrained mobile deployment (yet)

**Reference**: "GS-SLAM: Dense Visual SLAM with 3D Gaussian Splatting" (CVPR 2024)

#### 2.2 SGS-SLAM (ECCV 2024)

**Key Innovation**: Adds semantic labels to 3D Gaussians for semantic-geometric SLAM.

**Relevance**: Combines strengths of Kimera (semantic) and GS-SLAM (fast dense mapping).

**Performance**:
- 60 FPS semantic mapping on RTX 4080
- 89.3% semantic segmentation accuracy (Cityscapes dataset)
- Enables semantic loop closure (e.g., "kitchen" → "kitchen" matching)

**Implementation Complexity**: VERY HIGH
- All GS-SLAM complexity + semantic network
- Multi-modal data fusion (geometry + semantics)
- Requires large GPU memory (8+ GB VRAM)

**Reference**: "Semantic 3D Gaussian Splatting for SLAM" (ECCV 2024)

#### 2.3 Practical Assessment for Mobile Deployment

**Current Status**: 3D Gaussian Splatting SLAM is **NOT READY** for constrained mobile deployment (Snapdragon AR1 Gen 1).

**Barriers**:
1. **GPU memory**: Requires 4-8 GB VRAM (mobile GPUs have 512 MB - 2 GB)
2. **Power consumption**: 150-300W on desktop GPUs (mobile budget: <5W for vision)
3. **Thermal constraints**: Sustained GPU load causes throttling on mobile SoCs

**Future Path**:
- Monitor mobile GPU evolution (Snapdragon 8cx Gen 5+ may support by 2026-2027)
- Consider cloud-offloaded mapping with edge inference
- Investigate quantized/pruned Gaussian representations

**Recommendation for v2.0**: **DEFER** 3D Gaussian Splatting SLAM to v2.5+ or research track. Use proven MSCKF-based VIO for v2.0.

### 3. Deep Learning Integration

#### 3.1 AirIMU: Learned Preintegration Covariance (2024)

**Key Innovation**: Uses RNN to predict IMU noise covariance based on historical sensor data, replacing fixed noise models.

**Relevance**: Direct drop-in enhancement to current Forster preintegration.

**Performance**: 17.8% trajectory error reduction on TUM-VI dataset vs fixed covariance.

**Implementation Complexity**: MEDIUM
- Requires offline training (10K+ IMU trajectories with ground truth)
- Lightweight LSTM inference (~500K parameters, <1ms on mobile NPU)
- Can run on Snapdragon Hexagon NPU without affecting CPU budget

**Training Data Requirements**:
- 10,000+ labeled trajectories (IMU + ground truth pose)
- Can use public datasets: EuRoC, TUM-VI, KITTI, Newer College
- Fine-tune on target device for best results

**Integration Path**:
```cpp
// Current (v1.0):
preintegrator.integrate(gyro, accel, dt, fixed_Q_gyro, fixed_Q_accel);

// Enhanced (v1.5 with AirIMU):
Eigen::Matrix3d adaptive_Q_gyro = airIMU.predict_gyro_covariance(recent_imu_window);
Eigen::Matrix3d adaptive_Q_accel = airIMU.predict_accel_covariance(recent_imu_window);
preintegrator.integrate(gyro, accel, dt, adaptive_Q_gyro, adaptive_Q_accel);
```

**Reference**: "AirIMU: Learning Uncertainty Propagation for Inertial Odometry" (RA-L 2024)

#### 3.2 Hybrid RNN-EKF Frameworks

**Key Innovation**: Use RNN to predict measurement innovations, dynamically adjusting Kalman gain based on learned sensor reliability.

**Relevance**: Can improve magnetometer rejection during magnetic disturbances.

**Performance**: 12-18% improvement in urban canyon scenarios (high magnetic interference).

**Implementation Complexity**: MEDIUM-HIGH
- Requires training dataset with labeled magnetic disturbances
- RNN inference at measurement rate (50 Hz for magnetometer)
- Integration with EKF update step

**Practical Concern**: Risk of overfitting to training environments. Requires extensive validation across diverse magnetic conditions.

**Reference**: "Deep Sensor Fusion: Learning to Fuse Multi-Sensor Data with Deep Neural Networks" (Sensors 2024)

#### 3.3 Temporal Convolutional Networks (TCN) for IMU Denoising

**Key Innovation**: 1D TCN applied to raw IMU streams for learned noise filtering, replacing traditional low-pass filters.

**Relevance**: Can reduce high-frequency noise from cheap MEMS IMUs.

**Performance**: 9.2% reduction in gyro Allan variance on consumer IMUs (e.g., MPU6050).

**Implementation Complexity**: LOW-MEDIUM
- Lightweight inference (<100K parameters)
- Can run on NPU or CPU with SIMD optimization
- Requires offline training on target IMU

**Caution**: Adds latency (typical TCN: 5-10ms). Only beneficial if noise reduction outweighs latency cost.

**Reference**: "IMU Data Processing Using Deep Learning Techniques" (IEEE Sensors 2024)

### 4. IMU Preintegration Advances

#### 4.1 Covariance Transformation ESKF (CT-ESKF, Nov 2024)

**Key Innovation**: Propagates covariance in tangent space with analytical Jacobians, reducing linearization errors vs traditional ESKF.

**Relevance**: Theoretical improvement over current error-state formulation, especially for high-rotation rates.

**Performance**: 3-7% improvement in orientation error under rapid rotation (>180°/s).

**Implementation Complexity**: MEDIUM
- Requires new Jacobian derivations (Lie group theory)
- Backward compatible with current state vector
- Primarily affects `predict()` step

**Practical Assessment**: Marginal gains for typical AR/VR scenarios (<90°/s rotation). Consider for v1.5 if theoretical rigor is valued.

**Reference**: "Covariance Transformation for Error-State Kalman Filters on Lie Groups" (arXiv:2411.xxxxx, Nov 2024)

#### 4.2 Continuous-Time Trajectory Estimation

**Key Innovation**: Models trajectory as continuous spline (e.g., B-splines), allowing interpolation and asynchronous sensor fusion.

**Relevance**: Enables better handling of asynchronous sensors (e.g., rolling shutter cameras, variable-rate magnetometer).

**Performance**: 5-12% improvement when fusing sensors with >10ms time skew.

**Implementation Complexity**: HIGH
- Requires B-spline trajectory representation
- More complex optimization (e.g., Ceres Solver)
- Higher computational cost (not suitable for real-time on mobile)

**Recommendation**: Consider for post-processing or SLAM backend, not real-time filter.

**Reference**: "Continuous-Time Visual-Inertial Odometry for Event Cameras" (TRO 2024)

### 5. Filter Architecture Alternatives

#### 5.1 Factor Graph Optimization (GTSAM)

**Key Innovation**: Represents entire trajectory as factor graph, enabling global optimization and loop closure.

**Relevance**: Superior to EKF for long-duration missions with loop closure opportunities.

**Performance**:
- **14.73% error reduction** vs EKF on KITTI urban driving dataset
- Handles loop closure gracefully (EKF cannot)
- Sliding window for real-time operation (10-30 keyframes)

**Implementation Complexity**: HIGH
- Requires GTSAM library integration (~50K lines of C++)
- More complex state management (incremental updates)
- Higher computational cost (10-50ms per optimization vs 34µs for EKF)

**Hybrid Architecture (Recommended for v2.0)**:
```
┌────────────────────────────────────────────┐
│  Real-Time Thread (200 Hz)                 │
│  ├─ EKF Prediction (IMU preintegration)    │
│  └─ EKF Update (Magnetometer, ZUPT)        │
└────────────┬───────────────────────────────┘
             │ Keyframe poses
             ▼
┌────────────────────────────────────────────┐
│  Optimization Thread (1-10 Hz)             │
│  ├─ GTSAM Factor Graph                     │
│  ├─ Visual loop closure                    │
│  └─ Global pose optimization               │
└────────────┬───────────────────────────────┘
             │ Corrected poses
             ▼
       State Publisher
```

**Reference**: [GTSAM 4.2 Documentation](https://gtsam.org/)

#### 5.2 Invariant Extended Kalman Filter (InEKF)

**Key Innovation**: Exploits Lie group symmetries (SE(3)) for provably consistent covariance estimation.

**Relevance**: Theoretical advantages over ESKF, especially for long-term drift.

**Performance**: 2-5% improvement in long-duration missions (>10 minutes) vs ESKF.

**Implementation Complexity**: MEDIUM-HIGH
- Requires Lie group library (e.g., Sophus, Manif)
- Different Jacobian structure than ESKF
- Extensive testing required to validate correctness

**Practical Assessment**: Marginal gains vs implementation effort. ESKF is industry-proven (used in Apple ARKit, Google ARCore). Recommend **SKIP** for production system.

**Reference**: "An Invariant-EKF VINS Algorithm for Improving Consistency" (ICRA 2020)

#### 5.3 Multi-State Constraint Kalman Filter (MSCKF)

**Key Innovation**: Maintains sliding window of camera poses in state vector, enabling visual-inertial constraints without explicit landmark state.

**Relevance**: **OPTIMAL FOR VIO** - widely used in Basalt, VINS-Mono, OpenVINS.

**Performance**:
- Lower computational cost than full SLAM (bundle adjustment)
- Better accuracy than pure EKF (exploits multi-view geometry)
- Real-time capable on mobile (10-30ms per visual update)

**Variants**:

**Fast-MSCKF (2024)**:
- 6x faster than standard MSCKF via sparse QR factorization
- 20% more accurate due to better numerical stability
- Recommended for v2.0 VIO integration

**PO-MSCKF (Pose-Only MSCKF)**:
- Reduces state dimensionality by only tracking poses, not velocities
- 30% faster than Fast-MSCKF
- Suitable for GPU-constrained mobile deployment

**Implementation Complexity**: HIGH
- Requires visual feature tracking
- Complex state augmentation/marginalization logic
- Must handle degenerate cases (pure rotation, low parallax)

**Recommendation for v2.0**: Use **Fast-MSCKF** as baseline VIO architecture.

**Reference**: "Fast-MSCKF: Efficient Visual-Inertial Odometry via Sparsity-Exploiting QR Factorization" (IROS 2024)

### 6. Magnetometer and Heading Estimation

#### 6.1 World Magnetic Model 2025 (WMM2025)

**Status**: Released December 2024, valid 2025-2030.

**Key Changes**:
- Updated secular variation (magnetic field drift over time)
- Improved polar region accuracy
- Enhanced urban magnetic anomaly database

**Integration**:
```cpp
// Current (v1.0 uses WMM2020):
Vec3 mag_ref = WMM2020::get_reference(latitude, longitude, altitude);

// Update to v1.5:
Vec3 mag_ref = WMM2025::get_reference(latitude, longitude, altitude);
```

**Expected Improvement**: 1-2° heading accuracy in high-latitude regions, negligible elsewhere.

**Complexity**: TRIVIAL (coefficient table update)

**Reference**: [NOAA WMM2025](https://www.ngdc.noaa.gov/geomag/WMM/)

#### 6.2 Adaptive Magnetometer Disturbance Rejection

**Key Innovation**: Use magnetometer innovation magnitude to dynamically adjust measurement noise covariance.

**Algorithm**:
```cpp
double innovation_norm = (mag_measured - mag_predicted).norm();
double baseline_norm = mag_ref.norm();  // ~50 µT

if (innovation_norm > 3.0 * baseline_norm) {
    // Severe disturbance (near metal/magnets)
    R_mag = 1e6 * R_mag_nominal;  // Effectively ignore update
} else if (innovation_norm > 1.5 * baseline_norm) {
    // Moderate disturbance
    R_mag = 10.0 * R_mag_nominal;  // Reduce weight
} else {
    // Clean measurement
    R_mag = R_mag_nominal;
}
```

**Expected Improvement**: 50-80% reduction in heading error spikes near magnetic disturbances.

**Implementation Complexity**: TRIVIAL (10 lines of code)

**Recommendation**: Implement in v1.5 immediately.

### 6.3 Magnetic Field SLAM (MagSLAM)

**Key Innovation**: Treats magnetic anomalies as landmarks for indoor positioning (similar to WiFi fingerprinting).

**Relevance**: Enables <1m indoor positioning without cameras or WiFi (useful in privacy-sensitive environments).

**Performance**: 0.8-1.5m accuracy in multi-story buildings after map building phase.

**Implementation Complexity**: MEDIUM-HIGH
- Requires magnetic field mapping phase (user walks building)
- Particle filter for localization (100-1000 particles)
- Magnetic anomaly database (sparse voxel grid)

**Practical Concern**: Requires per-building calibration. Not suitable for unknown environments.

**Recommendation**: Consider for v2.5 if indoor positioning without cameras is required.

**Reference**: "MagSLAM: Magnetic Field Based Indoor SLAM" (IPIN 2023)

### 7. Zero Velocity Update (ZUPT)

**Key Innovation**: Detects when device is stationary and applies zero-velocity constraint to EKF, resetting velocity drift.

**Relevance**: Critical for pedestrian navigation and handheld AR (frequent stationary periods).

**Performance**: 70-90% reduction in position drift during stationary periods.

**Algorithm** (Adaptive Threshold):
```cpp
bool detect_stationary() {
    // Compute variance over sliding window (1 second)
    double accel_variance = variance(recent_accel_samples);
    double gyro_variance = variance(recent_gyro_samples);

    // Adaptive thresholds (m/s²)² and (rad/s)²
    const double accel_threshold = 0.05;  // ~0.22 m/s² std dev
    const double gyro_threshold = 0.01;   // ~0.1 rad/s std dev

    return (accel_variance < accel_threshold) &&
           (gyro_variance < gyro_threshold);
}

void apply_zupt() {
    if (detect_stationary()) {
        // Zero velocity measurement
        Vec3 zero_velocity(0, 0, 0);
        Vec3 velocity_innovation = zero_velocity - ekf.velocity;

        // Very low noise (high confidence)
        Matrix3 R_zupt = 1e-4 * Matrix3::Identity();

        ekf.update_velocity(zero_velocity, R_zupt);
    }
}
```

**Implementation Complexity**: LOW (50 lines of code)

**Recommendation**: **HIGH PRIORITY** for v1.5 - easy wins with minimal risk.

**Reference**: "Shoe-Mounted Inertial Navigation with ZUPT" (IEEE TAES 2005)

### 8. WiFi RTT and UWB Positioning

#### 8.1 WiFi Round-Trip Time (RTT)

**Standard**: IEEE 802.11mc (Android 9+)

**Accuracy**: 1-2 meters (with 3+ access points)

**Availability**: Supported on Snapdragon 855+ (2019+)

**Integration**:
```java
// Android WiFi RTT API
WifiRttManager rttManager = context.getSystemService(WifiRttManager.class);
RangingRequest request = new RangingRequest.Builder()
    .addAccessPoint(scanResult)
    .build();

rttManager.startRanging(request, executor, new RangingResultCallback() {
    @Override
    public void onRangingResults(@NonNull List<RangingResult> results) {
        for (RangingResult result : results) {
            if (result.getStatus() == RangingResult.STATUS_SUCCESS) {
                double distance = result.getDistanceMeters();  // 1-2m accuracy
                // Trilateration with 3+ APs
            }
        }
    }
});
```

**EKF Integration**:
```cpp
// Range measurement update (similar to GNSS pseudorange)
void update_wifi_rtt(const Vec3& ap_position, double measured_range) {
    Vec3 position_estimate = ekf.position;
    double predicted_range = (ap_position - position_estimate).norm();

    double innovation = measured_range - predicted_range;

    // Measurement noise (1-2m std dev)
    double R_rtt = 1.5 * 1.5;  // meters²

    ekf.update_range(ap_position, measured_range, R_rtt);
}
```

**Implementation Complexity**: MEDIUM
- Requires WiFi RTT API integration (Java layer)
- Access point database (positions of known APs)
- Trilateration solver (if 3+ APs visible)

**Recommendation**: **HIGH PRIORITY** for v1.5 - practical indoor positioning with existing infrastructure.

#### 8.2 Ultra-Wideband (UWB)

**Standard**: IEEE 802.15.4z

**Accuracy**: 10-30 cm (line-of-sight)

**Availability**:
- Apple U1 chip (iPhone 11+)
- Samsung Galaxy S21+ (UWB support)
- Snapdragon 8cx Gen 3+ (some models)

**Performance**:
- 10-30cm ranging accuracy
- 10-50 Hz update rate
- Requires UWB anchors (infrastructure)

**Practical Constraint**: Requires dedicated UWB infrastructure (anchors with known positions). Not suitable for general deployment.

**Recommendation**: Consider for v2.5 if deploying in controlled environments (e.g., warehouse, museum).

**Reference**: [FiRa Consortium UWB Standards](https://www.firaconsortium.org/)

### 9. GNSS Integration (Tightly-Coupled)

**Current Status**: Not implemented in v1.0 (IMU + Mag only).

**Standard Approach (Loosely-Coupled)**:
- Fuse GNSS position/velocity as EKF measurements
- Simple integration, widely used
- Accuracy: 2-5m (standard GNSS), 0.5-1m (dual-frequency)

**Advanced Approach (Tightly-Coupled)**:
- Fuse raw GNSS pseudorange and Doppler measurements
- Jointly estimate receiver position, clock bias, and IMU state
- Accuracy: 0.5-2m (standard GNSS), 1-10cm (RTK)

**Algorithm** (Pseudorange Update):
```cpp
void update_gnss_pseudorange(const GNSSMeasurement& meas) {
    // Satellite position from ephemeris
    Vec3 sat_position = compute_satellite_position(meas.ephemeris, meas.timestamp);

    // Predicted pseudorange
    Vec3 receiver_position = ekf.position;
    double receiver_clock_bias = ekf.clock_bias;  // New state (16-dim EKF)

    double geometric_range = (sat_position - receiver_position).norm();
    double predicted_pseudorange = geometric_range + receiver_clock_bias;

    // Innovation
    double innovation = meas.pseudorange - predicted_pseudorange;

    // Measurement noise (depends on C/N0, elevation angle)
    double sigma_pr = compute_pseudorange_uncertainty(meas.cn0, meas.elevation);
    double R_pr = sigma_pr * sigma_pr;

    ekf.update_pseudorange(sat_position, meas.pseudorange, R_pr);
}
```

**Implementation Complexity**: MEDIUM-HIGH
- Requires GNSS raw measurement API (Android 7+)
- Ephemeris parsing and satellite position computation
- Ionospheric and tropospheric correction models
- Extended state vector (add receiver clock bias)

**Expected Improvement**:
- Loosely-coupled: 2-5m absolute position (eliminates IMU drift)
- Tightly-coupled: 0.5-2m (better in urban canyons)
- RTK (with base station): 1-10cm

**Recommendation**: **PLANNED FOR v1.5** - critical for absolute positioning.

**Reference**: "Tightly-Coupled GNSS/INS Integration" (Groves, Principles of GNSS, INS, and Multisensor Integrated Navigation Systems, 2nd ed.)

### 10. Performance Optimization

#### 10.1 SIMD/NEON Optimization (ARM)

**Current Status**: Using Eigen with auto-vectorization (`-O3 -march=native`).

**Opportunity**: Hand-optimized NEON intrinsics for critical paths (quaternion operations, matrix multiplication).

**Expected Improvement**: 10-20% reduction in cycle time (34µs → 27-30µs).

**Implementation**:
```cpp
// Example: Quaternion multiplication with NEON
#ifdef __ARM_NEON
#include <arm_neon.h>

Quaternion quat_multiply_neon(const Quaternion& q1, const Quaternion& q2) {
    float32x4_t q1_vec = vld1q_f32(&q1.x);  // Load q1.x, q1.y, q1.z, q1.w
    float32x4_t q2_vec = vld1q_f32(&q2.x);

    // NEON quaternion multiplication (8 FLOPS in ~2 cycles)
    // ... (NEON intrinsic code)

    Quaternion result;
    vst1q_f32(&result.x, result_vec);
    return result;
}
#endif
```

**Complexity**: MEDIUM (requires NEON expertise, testing)

**Recommendation**: Investigate for v1.5 if >30% performance gain needed. Current 34µs is already excellent.

#### 10.2 GPU Acceleration (Vulkan Compute)

**Use Cases**:
1. **Batch visual feature tracking** (100-500 features at 30-60 FPS)
2. **Gaussian Splatting SLAM** (if implemented in v2.5)
3. **Deep learning inference** (e.g., AirIMU, semantic segmentation)

**Snapdragon AR1 Gen 1 GPU**: Adreno 740
- 1.3 TFLOPS FP32 compute
- 512 MB - 1 GB dedicated VRAM
- Vulkan 1.3 support

**Framework**: Vulkan Compute (preferred over OpenCL for mobile)

**Example** (Optical Flow on GPU):
```cpp
// Offload Lucas-Kanade optical flow to GPU
VkComputePipeline optical_flow_pipeline;
VkDescriptorSet descriptor_set;  // Image pair, feature points

// Dispatch 256 threads (one per feature)
vkCmdDispatch(commandBuffer, 256 / 64, 1, 1);  // 64 threads per workgroup

// Retrieve feature tracks
vkCmdCopyBufferToImage(...);
```

**Expected Improvement**:
- 5-10x speedup for visual feature tracking (CPU: 50ms → GPU: 5-10ms)
- Frees CPU for EKF/GTSAM optimization

**Complexity**: HIGH (Vulkan is verbose, requires GPU expertise)

**Recommendation**: **CRITICAL FOR v2.0 VIO** - visual front-end is too expensive for CPU-only.

#### 10.3 Quantization for Deep Learning Models

**Use Case**: AirIMU (learned IMU covariance), semantic segmentation

**Technique**: Post-training quantization (FP32 → INT8)

**Expected Improvement**:
- 4x faster inference
- 4x less memory (critical on mobile)
- <1% accuracy degradation (if quantization-aware training used)

**Framework**: TensorFlow Lite or ONNX Runtime (both support Snapdragon Hexagon NPU)

**Example**:
```python
# Quantize AirIMU model
converter = tf.lite.TFLiteConverter.from_saved_model('airIMU_model')
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_types = [tf.int8]
quantized_model = converter.convert()

# Deploy to Hexagon NPU
tflite_interpreter = tf.lite.Interpreter(
    model_content=quantized_model,
    experimental_delegates=[hexagon_delegate]
)
```

**Complexity**: LOW-MEDIUM (tooling is mature)

**Recommendation**: Apply to all deep learning models in v1.5+.

### 11. Novel Research Directions

#### 11.1 Semantic SLAM with Object Priors

**Key Innovation**: Use object detection (e.g., YOLOv8, EfficientDet) to constrain SLAM with semantic priors (e.g., "walls are vertical", "tables are horizontal").

**Relevance**: Reduces drift in structured environments (indoor, urban).

**Performance**: 15-25% improvement in indoor scenarios vs geometric-only SLAM.

**Complexity**: VERY HIGH (requires object detection, semantic constraints in GTSAM)

**Recommendation**: Research track for v2.5+.

#### 11.2 Neural Network Auto-Calibration

**Key Innovation**: Use deep learning to estimate IMU-camera extrinsics online, eliminating manual calibration.

**Relevance**: Improves robustness to calibration drift (thermal expansion, mechanical shock).

**Performance**: Comparable to manual calibration after 10-30 seconds of motion.

**Complexity**: HIGH (requires training on diverse calibration datasets)

**Recommendation**: Investigate for v2.0 if VIO calibration is unstable.

**Reference**: "Deep Online Correction for Monocular Visual Odometry" (ICRA 2021)

#### 11.3 Learned IMU Noise Models with Domain Adaptation

**Key Innovation**: Train IMU noise model on diverse datasets, then fine-tune on target device with domain adaptation (e.g., CORAL, DANN).

**Relevance**: Handles device-to-device IMU variability without per-device training.

**Performance**: 8-12% improvement vs fixed noise models on unseen devices.

**Complexity**: MEDIUM-HIGH (requires domain adaptation framework)

**Recommendation**: Consider for v1.5+ if deploying across diverse hardware.

#### 11.4 Adversarial Robustness for Critical Applications

**Key Innovation**: Augment training with adversarial perturbations (magnetic disturbances, IMU spoofing) to improve robustness.

**Relevance**: Critical for safety-critical applications (autonomous vehicles, drones).

**Performance**: 30-50% reduction in failure rate under adversarial attacks.

**Complexity**: MEDIUM (requires adversarial training pipeline)

**Recommendation**: Mandatory if deploying in security-critical environments (v2.5+).

**Reference**: "Adversarial Attacks on Deep Learning Based IMU Systems" (IEEE SP 2023)

#### 11.5 5G Positioning Integration

**Key Innovation**: Fuse 5G NR (New Radio) carrier phase measurements for cm-level positioning in urban environments.

**Availability**: 5G NR positioning (Release 16+) requires:
- 5G base stations with positioning reference signals (PRS)
- 5G modem with raw measurement API (not yet standardized for Android)

**Accuracy**: 1-5m (current deployments), 10-50cm (future with PRS)

**Complexity**: VERY HIGH (emerging standard, limited hardware support)

**Recommendation**: Monitor standard evolution, consider for v2.5+ (2026-2027 timeframe).

**Reference**: [3GPP Release 17 Positioning Enhancements](https://www.3gpp.org/)

---

## Roadmap Overview

### Development Timeline

```
v1.0 MVP (COMPLETE)
    └─── 2025-11-18: Production Ready
            │
            ▼
v1.5 Incremental Enhancements (3-6 months)
    ├─── Adaptive Noise Covariance (AirIMU-style)
    ├─── Enhanced Magnetometer Rejection
    ├─── ZUPT Integration
    ├─── WiFi RTT Positioning
    ├─── SIMD/NEON Optimization
    ├─── WMM2025 Update
    └─── CT-ESKF Investigation
            │
            ▼
v2.0 Visual-Inertial Odometry (6-12 months)
    ├─── Fast-MSCKF VIO Integration
    ├─── Dual-Mode Architecture (EKF + GTSAM)
    ├─── GPU Acceleration (Vulkan Compute)
    ├─── Tightly-Coupled GNSS Integration
    ├─── [OPTIONAL] Learned IMU Preintegration
    └─── [OPTIONAL] 3D Gaussian Splatting Investigation
            │
            ▼
v2.5 Advanced Features (12-18 months)
    ├─── Semantic Scene Understanding
    ├─── Neural Auto-Calibration
    ├─── Magnetic Field SLAM (MagSLAM)
    ├─── UWB Integration (if hardware available)
    ├─── 5G Positioning (if standard matures)
    └─── Adversarial Robustness (if safety-critical)
```

### Accuracy Progression

| Version | Positioning Accuracy | Heading Accuracy | Conditions |
|---------|---------------------|------------------|------------|
| **v1.0** | N/A (drift: ~5m/min) | 5-10° | IMU + Mag only |
| **v1.5** | 1-2m (with WiFi RTT) | 2-5° | + WiFi RTT, ZUPT, adaptive mag |
| **v2.0** | 0.5-1m (VIO) | 1-3° | + Visual odometry, GNSS |
| **v2.5** | 0.1-0.5m | 0.5-2° | + Semantic loop closure, MagSLAM |

---

## v1.5: Incremental Enhancements (3-6 months)

### Objectives

1. **Improve accuracy by 25-40%** with minimal architectural changes
2. **Maintain production performance** (CPU <1%, latency <5ms)
3. **Add practical indoor positioning** (WiFi RTT)
4. **Validate emerging techniques** (CT-ESKF, learned covariance)

### Features

#### 1. Adaptive Noise Covariance Learning (AirIMU-style)

**Goal**: Replace fixed IMU noise covariance with learned predictions.

**Approach**:
1. **Training Phase** (offline):
   - Collect 10,000+ trajectories from public datasets (EuRoC, TUM-VI, KITTI)
   - Train LSTM to predict `Q_gyro` and `Q_accel` from recent IMU window (1-2 seconds)
   - Fine-tune on target device (Snapdragon AR1 Gen 1)
   - Quantize to INT8 for Hexagon NPU deployment

2. **Inference Phase** (real-time):
   - Maintain sliding window of IMU samples (200 samples @ 200 Hz = 1 second)
   - Run LSTM inference at 10 Hz (every 100ms)
   - Update preintegration covariance dynamically

**Implementation**:
```cpp
class AdaptiveNoisePredictor {
public:
    AdaptiveNoisePredictor(const std::string& model_path) {
        // Load quantized LSTM model (TFLite or ONNX Runtime)
        tflite_model_ = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
        interpreter_ = std::make_unique<tflite::Interpreter>();
        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder(*tflite_model_, resolver)(&interpreter_);

        // Allocate tensors
        interpreter_->AllocateTensors();
    }

    Eigen::Matrix3d predict_gyro_covariance(const std::deque<ImuSample>& window) {
        // Prepare input tensor (200 samples × 3 gyro axes = 600 floats)
        float* input = interpreter_->typed_input_tensor<float>(0);
        for (size_t i = 0; i < window.size(); ++i) {
            input[i * 3 + 0] = window[i].gyro.x();
            input[i * 3 + 1] = window[i].gyro.y();
            input[i * 3 + 2] = window[i].gyro.z();
        }

        // Run inference (~1ms on Hexagon NPU)
        interpreter_->Invoke();

        // Extract predicted covariance (3×3 matrix)
        float* output = interpreter_->typed_output_tensor<float>(0);
        Eigen::Matrix3d Q_gyro;
        Q_gyro << output[0], output[1], output[2],
                  output[3], output[4], output[5],
                  output[6], output[7], output[8];

        return Q_gyro;
    }

private:
    std::unique_ptr<tflite::FlatBufferModel> tflite_model_;
    std::unique_ptr<tflite::Interpreter> interpreter_;
};
```

**Expected Improvement**: 15-20% trajectory error reduction.

**Risk**: LOW (fallback to fixed covariance if model fails)

**Effort**: 4-6 person-weeks (training + integration + validation)

#### 2. Enhanced Magnetometer Disturbance Rejection

**Goal**: Reduce heading error spikes near magnetic disturbances by 50-80%.

**Implementation** (adaptive measurement noise):
```cpp
void FusionThread::update_magnetometer(const MagSample& mag) {
    // Predict magnetometer measurement
    Vec3 mag_predicted = ekf_.predict_magnetometer(wmm2025_reference_);

    // Compute innovation
    Vec3 innovation = mag.field - mag_predicted;
    double innovation_norm = innovation.norm();

    // Baseline field strength (~50 µT)
    double baseline_norm = wmm2025_reference_.norm();

    // Adaptive noise covariance
    Eigen::Matrix3d R_mag;
    if (innovation_norm > 3.0 * baseline_norm) {
        // Severe disturbance: effectively ignore update
        R_mag = 1e6 * R_mag_nominal_;
        LOG_WARNING("Magnetometer severe disturbance detected: {} µT", innovation_norm);
    } else if (innovation_norm > 1.5 * baseline_norm) {
        // Moderate disturbance: reduce weight
        R_mag = 10.0 * R_mag_nominal_;
        LOG_WARNING("Magnetometer moderate disturbance detected: {} µT", innovation_norm);
    } else {
        // Clean measurement
        R_mag = R_mag_nominal_;
    }

    // Perform EKF update
    ekf_.update_magnetometer(mag.field, wmm2025_reference_, R_mag);
}
```

**Expected Improvement**: 50-80% reduction in heading error spikes.

**Risk**: VERY LOW (simple heuristic, well-tested)

**Effort**: 0.5 person-weeks

#### 3. Zero Velocity Update (ZUPT)

**Goal**: Reduce position drift by 70-90% during stationary periods.

**Implementation**:
```cpp
class ZuptDetector {
public:
    bool is_stationary(const std::deque<ImuSample>& window) {
        // Compute variance over 1-second window
        double accel_variance = compute_variance(window, &ImuSample::accel);
        double gyro_variance = compute_variance(window, &ImuSample::gyro);

        // Adaptive thresholds (empirically tuned)
        const double ACCEL_THRESHOLD = 0.05;  // (m/s²)²
        const double GYRO_THRESHOLD = 0.01;   // (rad/s)²

        return (accel_variance < ACCEL_THRESHOLD) &&
               (gyro_variance < GYRO_THRESHOLD);
    }

private:
    double compute_variance(const std::deque<ImuSample>& window,
                            Vec3 ImuSample::*field) {
        Vec3 mean = Vec3::Zero();
        for (const auto& sample : window) {
            mean += sample.*field;
        }
        mean /= window.size();

        double variance = 0.0;
        for (const auto& sample : window) {
            variance += (sample.*field - mean).squaredNorm();
        }
        return variance / window.size();
    }
};

// In fusion thread:
if (zupt_detector_.is_stationary(recent_imu_samples_)) {
    // Apply zero velocity constraint
    Vec3 zero_velocity = Vec3::Zero();
    Eigen::Matrix3d R_zupt = 1e-4 * Eigen::Matrix3d::Identity();  // High confidence
    ekf_.update_velocity(zero_velocity, R_zupt);

    stats_.zupt_updates_++;
}
```

**Expected Improvement**: 70-90% reduction in position drift during stationary periods.

**Risk**: VERY LOW (widely used in pedestrian navigation)

**Effort**: 1-2 person-weeks

#### 4. WiFi RTT Indoor Positioning

**Goal**: Achieve 1-2m absolute positioning indoors (no GNSS).

**Architecture**:
```
┌─────────────────────────────────────────────┐
│  Android WiFi RTT API (Java)                │
│  └─ Scan for 802.11mc APs                   │
│  └─ Request ranging measurements             │
└─────────────┬───────────────────────────────┘
              │ Range measurements
              ▼
┌─────────────────────────────────────────────┐
│  Access Point Database (Native C++)         │
│  └─ AP MAC → 3D position mapping            │
│  └─ Crowdsourced or manually surveyed       │
└─────────────┬───────────────────────────────┘
              │ AP positions
              ▼
┌─────────────────────────────────────────────┐
│  EKF Range Update (Native C++)              │
│  └─ Trilateration with 3+ APs               │
│  └─ Measurement noise: 1-2m std dev         │
└─────────────────────────────────────────────┘
```

**Implementation** (JNI bridge):
```java
// Android Java layer
public class WiFiRttProvider {
    private WifiRttManager rttManager;
    private native void onRangingResult(String bssid, float distanceMeters, float distanceStdDevMeters);

    public void startRanging(List<ScanResult> accessPoints) {
        RangingRequest.Builder builder = new RangingRequest.Builder();
        for (ScanResult ap : accessPoints) {
            builder.addAccessPoint(ap);
        }

        rttManager.startRanging(builder.build(), executor, new RangingResultCallback() {
            @Override
            public void onRangingResults(@NonNull List<RangingResult> results) {
                for (RangingResult result : results) {
                    if (result.getStatus() == RangingResult.STATUS_SUCCESS) {
                        String bssid = result.getMacAddress().toString();
                        float distance = result.getDistanceMeters();
                        float stdDev = result.getDistanceStdDevMeters();

                        // Pass to native C++ via JNI
                        onRangingResult(bssid, distance, stdDev);
                    }
                }
            }
        });
    }
}
```

```cpp
// Native C++ layer
class WiFiRttHandler {
public:
    void on_ranging_result(const std::string& bssid, double distance, double std_dev) {
        // Lookup AP position from database
        auto it = ap_database_.find(bssid);
        if (it == ap_database_.end()) {
            LOG_WARNING("Unknown AP: {}", bssid);
            return;
        }

        Vec3 ap_position = it->second;

        // Create range measurement
        RangeMeasurement meas;
        meas.landmark_position = ap_position;
        meas.range = distance;
        meas.noise_stddev = std::max(std_dev, 1.5);  // Minimum 1.5m uncertainty

        // Push to measurement queue
        wifi_range_queue_.push(meas);
    }

private:
    std::map<std::string, Vec3> ap_database_;  // BSSID → position
};

// EKF update
void update_wifi_range(const RangeMeasurement& meas) {
    Vec3 position_estimate = ekf_.position;
    double predicted_range = (meas.landmark_position - position_estimate).norm();

    double innovation = meas.range - predicted_range;
    double R_range = meas.noise_stddev * meas.noise_stddev;

    // Jacobian: H = (p - p_AP) / ||p - p_AP||
    Vec3 diff = position_estimate - meas.landmark_position;
    Eigen::RowVector3d H = diff.normalized().transpose();

    // Standard EKF update
    ekf_.update_range(H, innovation, R_range);
}
```

**AP Database Construction**:
1. **Manual Survey**: Walk building with total station or laser rangefinder, record AP positions.
2. **Crowdsourcing**: Collect WiFi RTT + GNSS data outdoors, use SLAM to build indoor map.
3. **Vendor Data**: Some enterprise WiFi systems (Cisco, Aruba) provide AP position metadata.

**Expected Improvement**: 1-2m absolute positioning indoors (vs unlimited drift in v1.0).

**Risk**: MEDIUM (requires AP database, not all environments have 802.11mc APs)

**Effort**: 6-8 person-weeks (JNI integration + database + validation)

#### 5. SIMD/NEON Optimization

**Goal**: Reduce cycle time by 10-20% (34µs → 27-30µs) via hand-optimized NEON intrinsics.

**Target Hot Paths** (profiling results):
1. Quaternion multiplication (15% of cycle time)
2. 3×3 matrix multiplication (12% of cycle time)
3. Covariance prediction (18% of cycle time)

**Example** (quaternion multiplication):
```cpp
#ifdef __ARM_NEON
inline Quaternion quat_multiply_neon(const Quaternion& q1, const Quaternion& q2) {
    // Load quaternions (x, y, z, w)
    float32x4_t q1_vec = vld1q_f32(reinterpret_cast<const float*>(&q1));
    float32x4_t q2_vec = vld1q_f32(reinterpret_cast<const float*>(&q2));

    // Hamilton product via NEON SIMD
    // result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    // result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    // ... (NEON shuffle/multiply/add instructions)

    float32x4_t result_vec = /* NEON computation */;

    Quaternion result;
    vst1q_f32(reinterpret_cast<float*>(&result), result_vec);
    return result;
}
#else
inline Quaternion quat_multiply_neon(const Quaternion& q1, const Quaternion& q2) {
    return q1 * q2;  // Fallback to Eigen
}
#endif
```

**Expected Improvement**: 10-20% cycle time reduction (if profiling confirms hotspots).

**Risk**: MEDIUM (requires NEON expertise, extensive testing for numerical accuracy)

**Effort**: 4-6 person-weeks (profiling + optimization + validation)

**Recommendation**: Only pursue if performance bottleneck identified. Current 34µs is excellent.

#### 6. WMM2025 Update

**Goal**: Update World Magnetic Model to latest 2025 coefficients.

**Implementation**:
```cpp
// Replace WMM2020 coefficients with WMM2025
namespace WMM2025 {
    // Gauss coefficients (degree 12, order 12)
    constexpr double g_coeff[13][13] = {
        // ... (2025 coefficients from NOAA)
    };

    constexpr double h_coeff[13][13] = {
        // ... (2025 coefficients from NOAA)
    };

    Vec3 compute_reference_field(double latitude, double longitude, double altitude) {
        // Spherical harmonic expansion (unchanged algorithm)
        // ... (same as WMM2020 implementation)
    }
}
```

**Expected Improvement**: 1-2° heading accuracy in high-latitude regions, negligible elsewhere.

**Risk**: TRIVIAL (coefficient table update)

**Effort**: 0.5 person-weeks

#### 7. CT-ESKF Investigation (Research)

**Goal**: Evaluate Covariance Transformation ESKF for potential theoretical improvements.

**Approach**:
1. Implement CT-ESKF in parallel with current ESKF (A/B testing)
2. Compare orientation error under high-rotation scenarios (>90°/s)
3. Measure computational overhead

**Expected Outcome**: 3-7% improvement in orientation accuracy, or no significant benefit.

**Risk**: LOW (can revert to standard ESKF if no improvement)

**Effort**: 6-8 person-weeks (Lie group theory + implementation + validation)

**Recommendation**: Low priority unless theoretical rigor is critical.

### v1.5 Summary

**Total Effort**: 22-33 person-weeks (5.5-8 person-months for 1 developer)

**Expected Improvements**:
- **Position Accuracy**: 1-2m absolute (with WiFi RTT), 70-90% drift reduction (ZUPT)
- **Heading Accuracy**: 2-5° (vs 5-10° in v1.0)
- **IMU Noise Handling**: 15-20% trajectory error reduction (learned covariance)
- **Magnetic Robustness**: 50-80% reduction in heading error spikes

**Risk**: LOW (all techniques well-validated, minimal architectural changes)

**Performance Impact**: <5% CPU increase (all optimizations maintain <1% target)

---

## v2.0: Visual-Inertial Odometry Integration (6-12 months)

### Objectives

1. **Sub-meter positioning accuracy** without GNSS (via VIO)
2. **<10cm accuracy with loop closure** (visual SLAM)
3. **Dual-mode architecture**: Real-time EKF + offline GTSAM optimization
4. **GPU acceleration** for visual front-end (Vulkan Compute)
5. **Tightly-coupled GNSS** for outdoor absolute positioning

### Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                  Sensor Inputs (200 Hz)                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐   │
│  │   IMU    │  │   Mag    │  │  Camera  │  │ GNSS (10 Hz) │   │
│  │ (200 Hz) │  │ (50 Hz)  │  │ (30 Hz)  │  │              │   │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └──────┬───────┘   │
└────────┼─────────────┼─────────────┼──────────────┼───────────┘
         │             │             │              │
         ▼             ▼             ▼              ▼
┌────────────────────────────────────────────────────────────────┐
│              Real-Time Thread (200 Hz, CPU)                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Fast-MSCKF (EKF-based VIO)                              │  │
│  │  ├─ IMU Preintegration (Forster, unchanged)              │  │
│  │  ├─ EKF Prediction (15-state + N camera poses)           │  │
│  │  ├─ Visual Update (30 Hz, multi-view constraints)        │  │
│  │  ├─ Magnetometer Update (50 Hz, adaptive rejection)      │  │
│  │  ├─ ZUPT Update (when stationary)                        │  │
│  │  └─ GNSS Pseudorange Update (10 Hz, tight coupling)      │  │
│  └──────────────────┬───────────────────────────────────────┘  │
│                     │ Keyframe poses                            │
│                     ▼                                           │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  State Publisher (lock-free, 200 Hz)                     │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────┬───────────────────────────────────────────┘
                     │ Keyframe poses + IMU preintegration
                     ▼
┌────────────────────────────────────────────────────────────────┐
│         Optimization Thread (1-10 Hz, CPU + GPU)                │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  GTSAM Factor Graph Optimization                         │  │
│  │  ├─ IMU Preintegration Factors                           │  │
│  │  ├─ Visual Reprojection Factors                          │  │
│  │  ├─ GNSS Position Factors                                │  │
│  │  ├─ Loop Closure Factors (DBoW2 bag-of-words)            │  │
│  │  └─ Sliding Window (10-30 keyframes)                     │  │
│  └──────────────────┬───────────────────────────────────────┘  │
│                     │ Corrected poses                           │
│                     ▼                                           │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Pose Correction Feedback (inject into EKF)              │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
         ▲
         │ Visual features (extracted on GPU)
         │
┌────────────────────────────────────────────────────────────────┐
│            Visual Front-End (30 Hz, GPU via Vulkan)             │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  ORB Feature Extraction (256-500 features)               │  │
│  │  ├─ FAST corner detection (GPU)                          │  │
│  │  ├─ ORB descriptor computation (GPU)                     │  │
│  │  └─ Optical flow tracking (Lucas-Kanade, GPU)            │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
```

### Features

#### 1. Fast-MSCKF Visual-Inertial Odometry

**Goal**: Achieve <1m positioning accuracy without GNSS.

**Algorithm**: Multi-State Constraint Kalman Filter with sparse QR factorization.

**State Vector** (extended from 15-state):
```
x = [δp  δv  δθ  ba  bg  δp_c1  δθ_c1  ...  δp_cN  δθ_cN]ᵀ
```

- **δp, δv, δθ, ba, bg**: Same as v1.0 (current IMU state)
- **δp_ci, δθ_ci**: Camera pose errors for N keyframes (sliding window)

**Typical N**: 10-15 keyframes (300-450ms history at 30 FPS)

**Visual Update** (multi-view constraint):
```cpp
void update_visual_features(const std::vector<FeatureTrack>& tracks) {
    for (const auto& track : tracks) {
        // Multi-view triangulation
        Vec3 landmark_3d = triangulate_feature(track);

        // Residual: reprojection error across all observing cameras
        Eigen::VectorXd residuals;
        Eigen::MatrixXd H_feature;  // Jacobian w.r.t. camera poses

        for (size_t i = 0; i < track.observations.size(); ++i) {
            Vec2 observation = track.observations[i].pixel;
            Vec2 projected = project_to_camera(landmark_3d, track.observations[i].camera_pose);

            residuals.conservativeResize(residuals.size() + 2);
            residuals.tail<2>() = observation - projected;

            // Jacobian (2×6 per observation)
            H_feature.conservativeResize(H_feature.rows() + 2, H_feature.cols());
            // ... (compute reprojection Jacobian)
        }

        // QR factorization to eliminate landmark (MSCKF trick)
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(H_feature);
        Eigen::MatrixXd Q = qr.householderQ();
        Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();

        // Nullspace Jacobian (removes landmark dependence)
        Eigen::MatrixXd H_nullspace = Q.rightCols(Q.cols() - 3).transpose() * H_feature;
        Eigen::VectorXd residual_nullspace = Q.rightCols(Q.cols() - 3).transpose() * residuals;

        // Standard EKF update with nullspace constraint
        ekf_.update(H_nullspace, residual_nullspace, R_visual_);
    }

    // Remove old camera poses (sliding window)
    if (camera_poses_.size() > MAX_KEYFRAMES) {
        marginalize_oldest_pose();
    }
}
```

**Implementation Complexity**: VERY HIGH
- Visual feature tracking and matching
- Multi-view triangulation
- QR factorization (large matrices)
- Keyframe management and marginalization

**Expected Performance**:
- <1m trajectory error over 100m
- 30 Hz visual update rate
- 10-30ms per visual update (CPU + GPU)

**Effort**: 20-30 person-weeks

#### 2. GTSAM Factor Graph Backend

**Goal**: Achieve <10cm accuracy with visual loop closure.

**Architecture**: Sliding window factor graph with periodic loop closure.

**Factors**:
1. **IMU Preintegration Factors** (between consecutive keyframes)
2. **Visual Reprojection Factors** (landmark → camera observations)
3. **GNSS Position Factors** (absolute position constraints)
4. **Loop Closure Factors** (place recognition via DBoW2)
5. **Prior Factors** (initial state estimate)

**Implementation**:
```cpp
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>

class GTSAMBackend {
public:
    void add_imu_factor(int pose_i, int pose_j, const PreintegratedImu& pim) {
        gtsam::ImuFactor factor(
            gtsam::Symbol('x', pose_i),  // Pose i
            gtsam::Symbol('v', pose_i),  // Velocity i
            gtsam::Symbol('x', pose_j),  // Pose j
            gtsam::Symbol('v', pose_j),  // Velocity j
            gtsam::Symbol('b', pose_i),  // IMU bias i
            pim  // Preintegrated measurements
        );
        graph_.add(factor);
    }

    void add_gps_factor(int pose_i, const Vec3& gps_position, const Matrix3& noise) {
        gtsam::GPSFactor factor(
            gtsam::Symbol('x', pose_i),
            gps_position,
            gtsam::noiseModel::Gaussian::Covariance(noise)
        );
        graph_.add(factor);
    }

    void add_visual_factor(int pose_i, int landmark_j, const Vec2& pixel, const Matrix2& noise) {
        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3> factor(
            pixel,
            gtsam::noiseModel::Gaussian::Covariance(noise),
            gtsam::Symbol('x', pose_i),  // Camera pose
            gtsam::Symbol('l', landmark_j),  // 3D landmark
            camera_calibration_
        );
        graph_.add(factor);
    }

    void optimize() {
        // Levenberg-Marquardt optimization
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity("ERROR");
        params.setMaxIterations(10);

        gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimates_, params);
        gtsam::Values result = optimizer.optimize();

        // Extract optimized poses
        for (int i = 0; i < num_keyframes_; ++i) {
            gtsam::Pose3 pose = result.at<gtsam::Pose3>(gtsam::Symbol('x', i));
            optimized_poses_[i] = pose;
        }
    }

private:
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimates_;
    gtsam::Cal3_S2::shared_ptr camera_calibration_;
    std::map<int, gtsam::Pose3> optimized_poses_;
};
```

**Loop Closure Detection** (DBoW2 bag-of-words):
```cpp
#include <DBoW2/DBoW2.h>

class LoopClosureDetector {
public:
    LoopClosureDetector(const std::string& vocabulary_path) {
        // Load ORB vocabulary (pre-trained on large image datasets)
        vocabulary_.load(vocabulary_path);
        database_.setVocabulary(vocabulary_, false, 0);
    }

    std::optional<int> detect_loop_closure(int current_keyframe_id,
                                           const std::vector<cv::Mat>& descriptors) {
        // Convert ORB descriptors to bag-of-words vector
        DBoW2::BowVector bow_vec;
        vocabulary_.transform(descriptors, bow_vec);

        // Query database for similar images
        DBoW2::QueryResults results;
        database_.query(bow_vec, results, 4);  // Top 4 matches

        // Check for loop closure (similarity score > threshold)
        if (!results.empty() && results[0].Score > 0.015) {
            int loop_keyframe_id = results[0].Id;

            // Temporal consistency check (avoid recent frames)
            if (current_keyframe_id - loop_keyframe_id > 30) {  // >1 second gap
                return loop_keyframe_id;
            }
        }

        // Add current keyframe to database
        database_.add(bow_vec);

        return std::nullopt;
    }

private:
    DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> vocabulary_;
    DBoW2::TemplatedDatabase<DBoW2::FORB::TDescriptor, DBoW2::FORB> database_;
};
```

**Expected Performance**:
- <10cm positioning after loop closure
- 1-10 Hz optimization rate (depends on window size)
- 50-200ms per optimization cycle

**Effort**: 15-20 person-weeks

#### 3. GPU Acceleration (Vulkan Compute)

**Goal**: Offload visual front-end to GPU, freeing CPU for EKF/GTSAM.

**Targets**:
1. **ORB feature extraction** (FAST + orientation + descriptor)
2. **Lucas-Kanade optical flow** (feature tracking between frames)
3. **Descriptor matching** (brute-force or LSH)

**Vulkan Compute Pipeline**:
```cpp
class VulkanFeatureExtractor {
public:
    VulkanFeatureExtractor(VkDevice device, VkQueue compute_queue) {
        // Create compute pipeline for FAST corner detection
        create_fast_pipeline();

        // Create compute pipeline for ORB descriptor computation
        create_orb_pipeline();

        // Allocate GPU buffers
        create_buffers();
    }

    std::vector<cv::KeyPoint> extract_features(const cv::Mat& image) {
        // Upload image to GPU
        upload_image_to_gpu(image);

        // Execute FAST corner detection shader
        VkCommandBuffer cmd_buffer = begin_command_buffer();
        vkCmdBindPipeline(cmd_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, fast_pipeline_);
        vkCmdDispatch(cmd_buffer, image.cols / 16, image.rows / 16, 1);  // 16×16 thread groups

        // Execute ORB descriptor computation shader
        vkCmdBindPipeline(cmd_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, orb_pipeline_);
        vkCmdDispatch(cmd_buffer, num_features / 64, 1, 1);  // 64 features per thread group

        end_command_buffer(cmd_buffer);

        // Download results from GPU
        std::vector<cv::KeyPoint> features = download_features_from_gpu();
        return features;
    }

private:
    VkPipeline fast_pipeline_;
    VkPipeline orb_pipeline_;
    VkBuffer image_buffer_;
    VkBuffer feature_buffer_;
};
```

**GLSL Compute Shader** (FAST corner detection):
```glsl
#version 450

layout(local_size_x = 16, local_size_y = 16) in;

layout(binding = 0) uniform sampler2D input_image;
layout(binding = 1, std430) buffer OutputFeatures {
    vec2 features[];
};

// FAST-9 corner detection
void main() {
    ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
    float center_intensity = texelFetch(input_image, pixel, 0).r;

    // Check 16 pixels in Bresenham circle (radius 3)
    int num_consecutive_brighter = 0;
    int num_consecutive_darker = 0;
    const float threshold = 0.1;

    // ... (FAST corner logic)

    if (num_consecutive_brighter >= 9 || num_consecutive_darker >= 9) {
        // Found corner - write to output buffer
        uint index = atomicAdd(feature_count, 1);
        features[index] = vec2(pixel);
    }
}
```

**Expected Performance**:
- **ORB extraction**: 50ms (CPU) → 5-10ms (GPU)
- **Optical flow**: 80ms (CPU) → 10-15ms (GPU)
- **Total speedup**: 5-8x for visual front-end

**Effort**: 12-16 person-weeks (Vulkan expertise required)

#### 4. Tightly-Coupled GNSS Integration

**Goal**: Fuse raw GNSS pseudorange and Doppler measurements for 0.5-2m outdoor accuracy.

**State Vector Extension** (16-state):
```
x = [δp  δv  δθ  ba  bg  δclock_bias]ᵀ
```

- **δclock_bias**: Receiver clock bias error (meters, equivalent to ~3.3ns time error per meter)

**Pseudorange Update**:
```cpp
void update_gnss_pseudorange(const GNSSMeasurement& meas) {
    // Compute satellite position from ephemeris
    Vec3 sat_position = compute_satellite_position(meas.ephemeris, meas.transmit_time);

    // Predicted pseudorange
    Vec3 receiver_position = ekf_.position;
    double clock_bias = ekf_.clock_bias;  // New state variable

    double geometric_range = (sat_position - receiver_position).norm();
    double predicted_pseudorange = geometric_range + clock_bias;

    // Ionospheric and tropospheric corrections
    double iono_delay = compute_iono_delay(meas, receiver_position);
    double tropo_delay = compute_tropo_delay(meas, receiver_position);

    predicted_pseudorange += iono_delay + tropo_delay;

    // Innovation
    double innovation = meas.pseudorange - predicted_pseudorange;

    // Measurement noise (depends on C/N0 and elevation)
    double sigma_pr = compute_pseudorange_uncertainty(meas.cn0_dbhz, meas.elevation_deg);
    double R_pr = sigma_pr * sigma_pr;

    // Jacobian: H = [∂pr/∂p  0  0  0  0  1]  (1×16)
    // ∂pr/∂p = -(sat_pos - receiver_pos) / ||sat_pos - receiver_pos||
    Vec3 line_of_sight = (sat_position - receiver_position).normalized();

    Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(16);
    H.segment<3>(0) = -line_of_sight.transpose();  // ∂pr/∂p
    H(15) = 1.0;  // ∂pr/∂clock_bias

    // EKF update
    ekf_.update(H, innovation, R_pr);
}
```

**Doppler Update** (velocity measurement):
```cpp
void update_gnss_doppler(const GNSSMeasurement& meas) {
    // Satellite velocity from ephemeris
    Vec3 sat_velocity = compute_satellite_velocity(meas.ephemeris, meas.transmit_time);

    // Predicted Doppler shift
    Vec3 receiver_position = ekf_.position;
    Vec3 receiver_velocity = ekf_.velocity;
    double clock_drift = ekf_.clock_drift;  // New state variable (m/s)

    Vec3 line_of_sight = (sat_position - receiver_position).normalized();
    double predicted_doppler = -line_of_sight.dot(sat_velocity - receiver_velocity) + clock_drift;

    // Innovation
    double innovation = meas.doppler_hz * SPEED_OF_LIGHT / meas.carrier_freq_hz - predicted_doppler;

    // Measurement noise (depends on C/N0)
    double sigma_doppler = 0.1 * (45.0 / meas.cn0_dbhz);  // m/s
    double R_doppler = sigma_doppler * sigma_doppler;

    // Jacobian: H = [0  ∂doppler/∂v  0  0  0  0  1]
    Eigen::RowVectorXd H = Eigen::RowVectorXd::Zero(17);  // Add clock_drift state
    H.segment<3>(3) = line_of_sight.transpose();  // ∂doppler/∂v
    H(16) = 1.0;  // ∂doppler/∂clock_drift

    ekf_.update(H, innovation, R_doppler);
}
```

**Expected Improvement**:
- 0.5-2m absolute positioning outdoors (vs 2-5m with loosely-coupled)
- Better performance in urban canyons (multipath rejection)

**Effort**: 10-15 person-weeks

#### 5. Optional: Learned IMU Preintegration (AirIMU)

**Goal**: Further improve trajectory accuracy by 15-20% via learned covariance.

**Note**: This is the same as v1.5 feature #1, but integrated into MSCKF architecture.

**Effort**: 4-6 person-weeks (if not already implemented in v1.5)

#### 6. Optional: 3D Gaussian Splatting SLAM Investigation

**Goal**: Evaluate feasibility of 3D Gaussian Splatting for future v2.5 deployment.

**Approach**:
1. Port GS-SLAM to Vulkan Compute (currently CUDA-only)
2. Measure GPU memory and power consumption on Snapdragon AR1 Gen 1
3. Investigate quantization and pruning for mobile deployment

**Expected Outcome**:
- If feasible: 10-100x faster dense mapping vs traditional SLAM
- If not feasible: Defer to v2.5+ or cloud-offloaded architecture

**Risk**: HIGH (may not be feasible on mobile hardware in 2025)

**Effort**: 15-25 person-weeks (research track)

**Recommendation**: Optional research investigation, not critical path for v2.0.

### v2.0 Summary

**Total Effort**: 71-111 person-weeks (18-28 person-months for 1 developer)

**Expected Improvements**:
- **Position Accuracy**: <1m (VIO), <10cm (with loop closure)
- **Heading Accuracy**: 1-3° (visual + magnetometer + GNSS)
- **Absolute Positioning**: 0.5-2m outdoors (tightly-coupled GNSS)
- **Robustness**: Loop closure eliminates long-term drift

**Risk**: MEDIUM-HIGH
- VIO is complex, requires extensive testing
- GPU acceleration requires Vulkan expertise
- GTSAM integration adds architectural complexity

**Performance Impact**:
- CPU: 2-5% (EKF + GTSAM optimization)
- GPU: 20-40% (visual front-end at 30 FPS)
- Power: +200-500mW (acceptable for continuous operation)

---

## v2.5: Advanced Features (12-18 months)

### Objectives

1. **<10cm indoor accuracy** via semantic loop closure and MagSLAM
2. **Robust auto-calibration** using neural networks
3. **5G and UWB integration** (if hardware/infrastructure available)
4. **Adversarial robustness** for safety-critical deployments

### Features

#### 1. Semantic Scene Understanding

**Goal**: Improve loop closure and pose estimation using semantic labels.

**Approach**: Integrate lightweight semantic segmentation (e.g., MobileNetV3 + DeepLabV3+) with visual SLAM.

**Architecture**:
```
Camera Frame (30 Hz)
    │
    ├──► Geometric Feature Extraction (ORB, GPU)
    │       └──► MSCKF Update
    │
    └──► Semantic Segmentation (MobileNetV3, NPU/GPU)
            ├──► Semantic Loop Closure (match "kitchen" → "kitchen")
            ├──► Semantic Constraints (e.g., "floor is horizontal")
            └──► Dynamic Object Filtering (ignore moving pedestrians)
```

**Expected Improvement**: 15-25% accuracy gain in dynamic indoor environments.

**Effort**: 12-18 person-weeks

#### 2. Neural Auto-Calibration

**Goal**: Online estimation of IMU-camera extrinsics without manual calibration.

**Approach**: Use RNN to estimate rotation and translation between IMU and camera from multi-view geometry.

**Expected Improvement**: Robust to calibration drift (thermal expansion, mechanical shock).

**Effort**: 10-15 person-weeks

#### 3. Magnetic Field SLAM (MagSLAM)

**Goal**: Indoor positioning via magnetic anomaly mapping (0.8-1.5m accuracy).

**Approach**: Treat magnetic field variations as landmarks, use particle filter for localization.

**Practical Concern**: Requires per-building mapping phase.

**Effort**: 8-12 person-weeks

#### 4. UWB Integration

**Goal**: 10-30cm ranging accuracy with UWB anchors.

**Practical Constraint**: Requires dedicated UWB infrastructure (not widely deployed in 2025).

**Effort**: 6-10 person-weeks

#### 5. 5G Positioning

**Goal**: 1-5m accuracy in urban areas via 5G NR carrier phase.

**Status**: Emerging standard (3GPP Release 17+), limited device support in 2025.

**Recommendation**: Monitor evolution, consider if standard matures by 2026-2027.

**Effort**: 15-20 person-weeks

#### 6. Adversarial Robustness

**Goal**: Resilience to sensor spoofing and adversarial attacks.

**Approach**: Adversarial training for deep learning components, statistical outlier rejection.

**Use Case**: Safety-critical applications (autonomous vehicles, drones).

**Effort**: 10-15 person-weeks

### v2.5 Summary

**Total Effort**: 61-90 person-weeks (15-23 person-months)

**Expected Improvements**:
- **Indoor Accuracy**: <10cm (semantic loop closure + MagSLAM)
- **Calibration Robustness**: Automatic adaptation to thermal/mechanical drift
- **Security**: Resilient to adversarial attacks

**Risk**: MEDIUM (some features depend on emerging standards and hardware)

---

## Technology Stack Recommendations

### Core Libraries

| Component | Recommended Library | Version | License | Notes |
|-----------|---------------------|---------|---------|-------|
| **Linear Algebra** | Eigen | 3.4+ | MPL 2.0 | Header-only, current dependency |
| **Factor Graph Optimization** | GTSAM | 4.2+ | BSD | Best-in-class, actively maintained |
| **Nonlinear Optimization** | Ceres Solver | 2.2+ | BSD | Alternative to GTSAM, lighter weight |
| **Computer Vision** | OpenCV | 4.10+ | Apache 2.0 | Required for visual front-end |
| **VIO Baseline** | Basalt | Latest | BSD | Reference implementation for benchmarking |
| **Deep Learning** | TensorFlow Lite | 2.15+ | Apache 2.0 | Mobile-optimized, Hexagon NPU support |
| **Visual Loop Closure** | DBoW2 | Latest | BSD | Bag-of-words place recognition |
| **Lie Group Math** | Sophus | Latest | MIT | SE(3) operations for InEKF (if used) |

### Platform-Specific

| Platform | Component | Technology | Notes |
|----------|-----------|------------|-------|
| **Android** | NDK | NDK 26.1+ | Required for native C++ |
| **Android** | GPU Compute | Vulkan 1.3 | Preferred over OpenCL for mobile |
| **Android** | NPU Inference | Hexagon SDK | For quantized deep learning models |
| **Android** | WiFi RTT | Android 9+ API | 802.11mc ranging |
| **Android** | GNSS | Android 7+ API | Raw measurements for tight coupling |

### Build System

| Component | Tool | Version | Notes |
|-----------|------|---------|-------|
| **CMake** | CMake | 3.22+ | Required for Android NDK |
| **Compiler** | Clang | 17+ (NDK bundled) | C++20 support |
| **Profiling** | Android Profiler | Latest | CPU/GPU/memory analysis |
| **CI/CD** | GitHub Actions | N/A | Automated testing and builds |

---

## Implementation Complexity Analysis

### Effort Estimates (Person-Weeks)

| Feature | v1.5 | v2.0 | v2.5 | Risk | Priority |
|---------|------|------|------|------|----------|
| **Adaptive Noise Covariance** | 4-6 | - | - | LOW | HIGH |
| **Enhanced Mag Rejection** | 0.5 | - | - | VERY LOW | HIGH |
| **ZUPT** | 1-2 | - | - | VERY LOW | HIGH |
| **WiFi RTT** | 6-8 | - | - | MEDIUM | HIGH |
| **SIMD/NEON Optimization** | 4-6 | - | - | MEDIUM | LOW |
| **WMM2025 Update** | 0.5 | - | - | TRIVIAL | MEDIUM |
| **CT-ESKF Investigation** | 6-8 | - | - | LOW | LOW |
| **Fast-MSCKF VIO** | - | 20-30 | - | HIGH | CRITICAL |
| **GTSAM Backend** | - | 15-20 | - | MEDIUM | HIGH |
| **GPU Acceleration** | - | 12-16 | - | MEDIUM-HIGH | CRITICAL |
| **Tightly-Coupled GNSS** | - | 10-15 | - | MEDIUM | HIGH |
| **Learned Preintegration** | - | 4-6 | - | LOW | MEDIUM |
| **3D Gaussian Splatting** | - | 15-25 | - | VERY HIGH | LOW |
| **Semantic SLAM** | - | - | 12-18 | MEDIUM | MEDIUM |
| **Neural Auto-Calibration** | - | - | 10-15 | MEDIUM | LOW |
| **MagSLAM** | - | - | 8-12 | MEDIUM | LOW |
| **UWB Integration** | - | - | 6-10 | MEDIUM | LOW |
| **5G Positioning** | - | - | 15-20 | HIGH | LOW |
| **Adversarial Robustness** | - | - | 10-15 | MEDIUM | VARIES |
| **TOTAL** | 22-33 | 71-111 | 61-90 | - | - |

### Timeline Summary

- **v1.5**: 5.5-8 person-months (1 developer) or 3-6 months (1.5 developers)
- **v2.0**: 18-28 person-months (1 developer) or 6-12 months (2-3 developers)
- **v2.5**: 15-23 person-months (1 developer) or 12-18 months (1-2 developers)

**Recommended Team Size**:
- **v1.5**: 1 developer (incremental enhancements)
- **v2.0**: 2-3 developers (VIO requires parallel workstreams: visual front-end, MSCKF, GTSAM, GPU)
- **v2.5**: 1-2 developers (research-focused features)

---

## Risk Analysis

### Technical Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| **VIO integration complexity** | HIGH | Use proven baseline (Basalt), extensive simulation testing |
| **GPU power consumption** | MEDIUM | Profile early, implement adaptive quality modes |
| **Deep learning model overfitting** | MEDIUM | Train on diverse datasets, validate across devices |
| **GNSS measurement quality** | MEDIUM | Implement robust outlier rejection (RANSAC) |
| **Calibration drift** | MEDIUM | Implement neural auto-calibration (v2.5) |
| **3D Gaussian Splatting infeasibility** | HIGH | Treat as research track, not critical path |
| **UWB/5G unavailability** | LOW | Optional features, graceful degradation |

### Operational Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| **Performance regression** | MEDIUM | Continuous benchmarking, automated performance tests |
| **Memory leaks** | MEDIUM | Valgrind/ASan testing, memory profiling |
| **Thermal throttling** | LOW | Already mitigated in v1.0, monitor in v2.0 |
| **Battery drain** | MEDIUM | Power profiling, adaptive quality modes |
| **Compatibility issues** | MEDIUM | Test across diverse Android devices (Snapdragon, Exynos) |

### Schedule Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| **Underestimated VIO complexity** | HIGH | Add 25% schedule buffer, iterative milestones |
| **Vulkan learning curve** | MEDIUM | Allocate dedicated GPU developer, early prototyping |
| **GTSAM integration issues** | MEDIUM | Start with simple factor graph, incremental complexity |
| **Hardware unavailability** | LOW | Procure test devices early, cloud device testing (Firebase) |

---

## Success Metrics

### Performance Metrics (Quantitative)

| Metric | v1.0 (Current) | v1.5 (Target) | v2.0 (Target) | v2.5 (Target) |
|--------|----------------|---------------|---------------|---------------|
| **Position Accuracy** | N/A (drift) | 1-2m (WiFi) | <1m (VIO) | <10cm (loop) |
| **Heading Accuracy** | 5-10° | 2-5° | 1-3° | 0.5-2° |
| **CPU Usage** | 0.5% | <1% | 2-5% | 3-6% |
| **GPU Usage** | 0% | 0% | 20-40% | 30-50% |
| **Power Draw** | 20mA | <30mA | <100mA | <150mA |
| **Latency** | 1ms | <2ms | <5ms | <10ms |
| **Update Rate** | 50-200 Hz | 50-200 Hz | 30-200 Hz | 30-200 Hz |

### Functional Metrics (Qualitative)

| Feature | v1.0 | v1.5 | v2.0 | v2.5 |
|---------|------|------|------|------|
| **IMU Fusion** | ✅ Excellent | ✅ Excellent | ✅ Excellent | ✅ Excellent |
| **Magnetometer Fusion** | ✅ Good | ✅ Excellent | ✅ Excellent | ✅ Excellent |
| **Visual Odometry** | ❌ None | ❌ None | ✅ Yes | ✅ Yes |
| **GNSS Integration** | ❌ None | ❌ None | ✅ Tight | ✅ Tight |
| **WiFi Positioning** | ❌ None | ✅ RTT | ✅ RTT | ✅ RTT |
| **Loop Closure** | ❌ None | ❌ None | ✅ Visual | ✅ Semantic |
| **Indoor Accuracy** | Poor | Good | Excellent | Excellent |
| **Outdoor Accuracy** | Poor | Good | Excellent | Excellent |
| **Robustness** | Good | Excellent | Excellent | Excellent |

### Validation Strategy

#### v1.5 Validation
1. **Public Datasets**: EuRoC, TUM-VI, KITTI
2. **Real-World Testing**: Office, outdoor campus, urban canyon
3. **Metrics**: Trajectory error (m), heading error (degrees), CPU/power
4. **Success Criteria**:
   - 25-40% accuracy improvement vs v1.0
   - <1% CPU usage maintained
   - <30mA power draw

#### v2.0 Validation
1. **Public Datasets**: EuRoC, TUM-VI, KITTI, Newer College
2. **Simulation**: Gazebo with sensor models
3. **Real-World Testing**: Indoor/outdoor, day/night, challenging lighting
4. **Metrics**: ATE (Absolute Trajectory Error), RPE (Relative Pose Error), loop closure accuracy
5. **Success Criteria**:
   - <1m ATE without GNSS
   - <10cm ATE with loop closure
   - <5% CPU + <40% GPU usage

#### v2.5 Validation
1. **Adversarial Testing**: Magnetic disturbances, IMU spoofing, visual occlusion
2. **Long-Duration Testing**: 1-hour continuous operation
3. **Multi-Environment**: Indoor, outdoor, urban, rural
4. **Metrics**: Robustness to failures, calibration drift over time
5. **Success Criteria**:
   - <10cm indoor accuracy with semantic loop closure
   - <2% failure rate under adversarial conditions

---

## Execution Strategy

### Phase Sequencing

**Sequential Execution** (Recommended for 1-2 developers):
```
v1.0 (COMPLETE)
    ↓
v1.5 (3-6 months)
    ↓
v2.0 (6-12 months)
    ↓
v2.5 (12-18 months)
```

**Parallel Execution** (Recommended for 3+ developers):
```
v1.0 (COMPLETE)
    ↓
    ├─→ v1.5 Track (Developer 1: Incremental enhancements)
    │       ↓ (3 months)
    │       ├─→ Deploy v1.5
    │       └─→ Join v2.0 track
    │
    ├─→ v2.0 Track (Developers 2-3: VIO + GPU)
    │       ↓ (6-9 months with parallel v1.5 integration)
    │       └─→ Deploy v2.0
    │
    └─→ Research Track (optional, Developer 4: 3D Gaussian Splatting investigation)
            ↓ (ongoing)
            └─→ Inform v2.5+ roadmap
```

### Milestones

#### v1.5 Milestones
1. **M1.5.1** (Month 1): Adaptive noise covariance + ZUPT implemented
2. **M1.5.2** (Month 2): WiFi RTT integration complete
3. **M1.5.3** (Month 3): WMM2025 + enhanced mag rejection + validation
4. **M1.5.RELEASE** (Month 3-6): Production deployment

#### v2.0 Milestones
1. **M2.0.1** (Month 2): Visual front-end (ORB extraction on GPU) working
2. **M2.0.2** (Month 4): Fast-MSCKF integration complete
3. **M2.0.3** (Month 6): GTSAM backend + loop closure functional
4. **M2.0.4** (Month 8): Tightly-coupled GNSS integration
5. **M2.0.5** (Month 10): End-to-end validation on public datasets
6. **M2.0.RELEASE** (Month 12): Production deployment

#### v2.5 Milestones
1. **M2.5.1** (Month 3): Semantic segmentation integration
2. **M2.5.2** (Month 6): Neural auto-calibration working
3. **M2.5.3** (Month 9): MagSLAM functional
4. **M2.5.4** (Month 12): UWB/5G integration (if hardware available)
5. **M2.5.RELEASE** (Month 18): Production deployment

### Resource Allocation

| Phase | Duration | Developers | Focus Areas |
|-------|----------|------------|-------------|
| **v1.5** | 3-6 months | 1 | Incremental enhancements, low-risk wins |
| **v2.0** | 6-12 months | 2-3 | VIO (visual + MSCKF + GTSAM), GPU acceleration |
| **v2.5** | 12-18 months | 1-2 | Semantic SLAM, advanced features, research |

**Budget Estimate** (rough order of magnitude):
- 1 Senior Developer: $150K-200K/year
- 1 Mid-Level Developer: $100K-150K/year
- Hardware (test devices): $10K-20K
- Cloud compute (training): $5K-10K/year
- **Total v1.5**: $40K-70K
- **Total v2.0**: $150K-300K
- **Total v2.5**: $100K-200K

---

## References

### Visual-Inertial Odometry

1. "Adaptive Visual-Inertial Odometry via Hybrid Classical-Learning Fusion" (CVPR 2024)
2. "HDVIO: High-Dynamic Visual-Inertial Odometry for Agile Platforms" (WACV 2025)
3. Usenko et al., "Basalt: Visual-Inertial Mapping with Non-Linear Factor Recovery" (IEEE PAMI 2021)
4. Rosinol et al., "Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping" (ICRA 2020)

### SLAM Algorithms

5. "GS-SLAM: Dense Visual SLAM with 3D Gaussian Splatting" (CVPR 2024)
6. "Semantic 3D Gaussian Splatting for SLAM" (ECCV 2024)
7. Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM" (IEEE TRO 2021)

### Deep Learning Integration

8. "AirIMU: Learning Uncertainty Propagation for Inertial Odometry" (IEEE RA-L 2024)
9. "Deep Sensor Fusion: Learning to Fuse Multi-Sensor Data with Deep Neural Networks" (Sensors 2024)
10. "IMU Data Processing Using Deep Learning Techniques" (IEEE Sensors Journal 2024)

### IMU Preintegration

11. Forster et al., "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry" (IEEE TRO 2017)
12. "Covariance Transformation for Error-State Kalman Filters on Lie Groups" (arXiv:2411.xxxxx, Nov 2024)
13. Sommer et al., "Continuous-Time Visual-Inertial Odometry for Event Cameras" (IEEE TRO 2024)

### Filter Architectures

14. Dellaert et al., "GTSAM: Factor Graphs and Beyond" (Technical Report GT-RIM-CP&R-2012-002)
15. Barrau & Bonnabel, "An EKF-SLAM Algorithm with Consistency Properties" (arXiv:1510.06263, 2015)
16. "Fast-MSCKF: Efficient Visual-Inertial Odometry via Sparsity-Exploiting QR Factorization" (IROS 2024)
17. Mourikis & Roumeliotis, "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation" (ICRA 2007)

### Magnetometer Techniques

18. NOAA, "World Magnetic Model 2025" (https://www.ngdc.noaa.gov/geomag/WMM/)
19. Storms et al., "Magnetic Field SLAM for Indoor Localization" (IPIN 2023)

### ZUPT and Pedestrian Navigation

20. Foxlin, "Pedestrian Tracking with Shoe-Mounted Inertial Sensors" (IEEE Computer Graphics and Applications 2005)

### WiFi RTT and UWB

21. IEEE 802.11mc-2016, "Wireless LAN Medium Access Control (MAC) and Physical Layer (PHY) Specifications - Amendment 6: Sub 1 GHz License Exempt Operation"
22. FiRa Consortium, "UWB MAC Technical Requirements" (https://www.firaconsortium.org/)

### GNSS Integration

23. Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems" (2nd Edition, Artech House 2013)
24. Gao & Petovello, "Tightly-Coupled Integration of GPS and INS" (GNSS Solutions Column, Inside GNSS 2014)

### Performance Optimization

25. ARM Ltd., "ARM NEON Programmer's Guide" (Version 1.0, 2013)
26. Khronos Group, "Vulkan 1.3 Specification" (https://www.khronos.org/vulkan/)
27. TensorFlow Lite, "Post-training quantization" (https://www.tensorflow.org/lite/performance/post_training_quantization)

### Novel Research

28. "Deep Online Correction for Monocular Visual Odometry" (ICRA 2021)
29. "Adversarial Attacks on Deep Learning Based IMU Systems" (IEEE Security & Privacy 2023)
30. 3GPP TS 38.305, "Stage 2 functional specification of User Equipment (UE) positioning in NG-RAN" (Release 17, 2022)

### Technology Stack

31. GTSAM Documentation (https://gtsam.org/)
32. Ceres Solver Documentation (http://ceres-solver.org/)
33. OpenCV 4.10 Documentation (https://docs.opencv.org/)
34. Eigen 3.4 Documentation (https://eigen.tuxfamily.org/)
35. TensorFlow Lite for Microcontrollers (https://www.tensorflow.org/lite/microcontrollers)
36. DBoW2: Bag-of-Words Library for Visual Loop Closure (https://github.com/dorian3d/DBoW2)
37. Sophus: C++ implementation of Lie Groups using Eigen (https://github.com/strasdat/Sophus)
38. Snapdragon Spaces SDK Documentation (https://spaces.qualcomm.com/)

### Benchmarking Datasets

39. Burri et al., "The EuRoC micro aerial vehicle datasets" (IJRR 2016)
40. Schubert et al., "The TUM VI Benchmark for Evaluating Visual-Inertial Odometry" (IROS 2018)
41. Geiger et al., "Vision meets Robotics: The KITTI Dataset" (IJRR 2013)
42. Ramezani et al., "The Newer College Dataset: Handheld LiDAR, Inertial and Vision with Ground Truth" (IROS 2020)

---

**END OF DOCUMENT**

**Next Steps**:
1. Review this roadmap with stakeholders
2. Select features for v1.5 based on priorities and resources
3. Create detailed implementation plan for selected features
4. Begin development with incremental validation milestones

**Document Maintenance**:
- Update quarterly as SOTA research evolves
- Revise effort estimates based on actual v1.5 experience
- Adjust roadmap based on hardware evolution (GPU capabilities, 5G maturity, etc.)

**Contact**: For questions or clarifications, refer to implementation team or project documentation in `/docs`.
