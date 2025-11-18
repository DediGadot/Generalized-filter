// Location Measurement Update for EKF
//
// Purpose: Position correction using Android Fused Location
// Pattern: Follows mag_update.hpp interface
//
// Measurement Model:
//   z = h(x) + v
//   where:
//     z = measured position in NED frame [m]
//     h(x) = p_n = predicted position from nominal state
//     v ~ N(0, R) = measurement noise (from accuracy estimate)
//
// Linearized Model:
//   δz = H * δx + v
//   where:
//     H = ∂h/∂x = [0, 0, I_3x3, 0, 0] (identity for position block)
//     R = diag(h_acc², h_acc², v_acc²) × adaptive_factor
//
// Sample Usage:
//   LocationUpdate loc_update;
//   LocationMeasurement loc = {...};
//
//   if (loc_update.update(state, loc)) {
//       // Update accepted
//       state.inject_error(state.error_state());
//       state.reset_error();
//   }

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

    // Diagnostics (mutable for const functions)
    mutable Vector3d last_innovation_;        ///< Last innovation vector
    mutable double last_adaptive_factor_;     ///< Last adaptive noise factor
    uint32_t consecutive_rejections_;         ///< Track consecutive outliers

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
