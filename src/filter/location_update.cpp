// Location Measurement Update - Implementation

#include "location_update.hpp"

namespace fusion {

LocationUpdate::LocationUpdate(double adaptive_noise_factor,
                               double max_horizontal_error)
    : update_(),
      adaptive_noise_factor_(adaptive_noise_factor),
      max_horizontal_error_(max_horizontal_error),
      last_innovation_(Vector3d::Zero()),
      last_adaptive_factor_(1.0),
      consecutive_rejections_(0) {
}

bool LocationUpdate::update(
    EkfState& state,
    const LocationMeasurement& location_meas,
    bool enable_gating) {

    // Step 1: Validate location quality
    if (!is_location_valid(location_meas)) {
        consecutive_rejections_++;
        return false;
    }

    // Step 2: Extract measured position (already in NED from JNI)
    Vector3d z_measured = location_meas.position_ned;

    // Step 3: Predict measurement (trivial - just the current position estimate)
    Vector3d h_x = state.position();

    // Step 4: Compute innovation
    Vector3d innovation = z_measured - h_x;
    last_innovation_ = innovation;

    // Step 5: Compute measurement Jacobian
    // H = [0_{3x3}, 0_{3x3}, I_{3x3}, 0_{3x3}, 0_{3x3}]
    // Only position block (columns 6-9) is identity, rest is zeros
    auto H = compute_jacobian();

    // Step 6: Compute adaptive noise covariance
    Matrix3d R = compute_adaptive_noise(location_meas);

    // Step 7: Perform generic measurement update with chi-square gating
    bool accepted = update_.update(
        state, innovation, H, R, enable_gating);

    if (accepted) {
        consecutive_rejections_ = 0;
    } else {
        consecutive_rejections_++;
    }

    return accepted;
}

bool LocationUpdate::is_location_valid(const LocationMeasurement& location_meas) const {
    // Check horizontal accuracy
    if (location_meas.horizontal_accuracy_m > max_horizontal_error_) {
        return false;
    }

    // Check vertical accuracy (allow 2× horizontal threshold)
    if (location_meas.vertical_accuracy_m > 2.0 * max_horizontal_error_) {
        return false;
    }

    // Check provider flags - reject if cell-only (no GPS or WiFi)
    // Bit flags: GPS=1, WiFi=2, Cell=4, BLE=8
    constexpr uint8_t GPS_FLAG = 1;
    constexpr uint8_t WIFI_FLAG = 2;

    bool has_gps_or_wifi = (location_meas.provider_flags & (GPS_FLAG | WIFI_FLAG)) != 0;
    if (!has_gps_or_wifi) {
        // Cell-only location is too inaccurate
        return false;
    }

    return true;
}

Matrix3d LocationUpdate::compute_adaptive_noise(
    const LocationMeasurement& location_meas) const {

    // Base noise from Android accuracy estimates
    double h_acc = static_cast<double>(location_meas.horizontal_accuracy_m);
    double v_acc = static_cast<double>(location_meas.vertical_accuracy_m);

    // Ensure minimum noise (avoid over-confidence)
    h_acc = std::max(h_acc, 1.0);  // Minimum 1m horizontal
    v_acc = std::max(v_acc, 2.0);  // Minimum 2m vertical

    // Adaptive factor: increase noise if accuracy is poor
    double adaptive_factor = 1.0;
    if (h_acc > 50.0) {
        // Poor accuracy - increase noise
        adaptive_factor = adaptive_noise_factor_;
    } else if (consecutive_rejections_ > 2) {
        // Multiple consecutive rejections - increase noise
        adaptive_factor = adaptive_noise_factor_;
    }

    last_adaptive_factor_ = adaptive_factor;

    // Build diagonal noise covariance matrix
    // R = diag(σ_n², σ_e², σ_d²) × adaptive_factor²
    Matrix3d R = Matrix3d::Zero();
    R(0, 0) = h_acc * h_acc * adaptive_factor * adaptive_factor;  // North variance
    R(1, 1) = h_acc * h_acc * adaptive_factor * adaptive_factor;  // East variance
    R(2, 2) = v_acc * v_acc * adaptive_factor * adaptive_factor;  // Down variance

    return R;
}

Eigen::Matrix<double, 3, 15> LocationUpdate::compute_jacobian() const {
    // Measurement Jacobian: H = ∂h/∂δx
    // h(x) = p_n (position)
    // δx = [δθ(3), δv(3), δp(3), δb_g(3), δb_a(3)]
    //
    // ∂h/∂δθ = 0 (position doesn't depend on orientation error)
    // ∂h/∂δv = 0 (position doesn't depend on velocity error)
    // ∂h/∂δp = I_{3x3} (position error directly affects measurement)
    // ∂h/∂δb_g = 0 (position doesn't depend on gyro bias)
    // ∂h/∂δb_a = 0 (position doesn't depend on accel bias)

    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();

    // Set position block to identity (columns 6-9 for δp)
    H(0, 6) = 1.0;  // ∂north/∂δp_north
    H(1, 7) = 1.0;  // ∂east/∂δp_east
    H(2, 8) = 1.0;  // ∂down/∂δp_down

    return H;
}

}  // namespace fusion
