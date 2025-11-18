// Magnetometer Measurement Update
//
// Purpose: Provides magnetometer heading correction for the error-state EKF
// Reference: "Quaternion kinematics for the error-state Kalman filter" - Joan Solà
//
// Key Features:
// - 3DOF magnetometer measurement (magnetic field vector)
// - World Magnetic Model (WMM2025) for reference field
// - Adaptive noise (increase R if magnetic disturbances detected)
// - Outlier rejection via chi-square gating
//
// Measurement Model:
//   z = h(x) + v
//   where:
//     z = measured magnetic field in body frame [μT]
//     h(x) = R_nb^T * m_n = predicted magnetic field in body frame
//     m_n = reference magnetic field in navigation frame (from WMM)
//     R_nb = rotation matrix from nominal quaternion
//     v ~ N(0, R) = measurement noise
//
// Linearized Measurement Model:
//   δz = H * δx + v
//   where:
//     H = ∂h/∂x = measurement Jacobian (3 × 15)
//     For magnetometer: only depends on orientation error
//     ∂h/∂δθ = [R_nb^T * m_n]_× (skew-symmetric matrix)
//     ∂h/∂(other states) = 0
//
// Adaptive Noise:
//   - Nominal noise: σ_mag² (from sensor specs, ~0.5 μT)
//   - If ||z|| deviates significantly from ||m_n||:
//     → Magnetic disturbance detected (e.g., near metal)
//     → Increase R to reduce trust in measurement
//   - Adaptive factor: k = 1 + α * (|deviation| / ||m_n||)
//   - Updated noise: R = k² * R_nominal
//
// Sample Usage:
//   MagUpdate mag_update;
//
//   // Set location for WMM (latitude, longitude, altitude)
//   mag_update.set_location(37.7749, -122.4194, 100.0);  // San Francisco
//
//   // Get measured field from sensor
//   Vector3d mag_body = magnetometer.calibrated_reading();
//
//   // Perform update (with adaptive noise and outlier rejection)
//   if (mag_update.update(state, mag_body)) {
//       // Update accepted
//       state.inject_error(state.error_state());
//       state.reset_error();
//   } else {
//       // Update rejected (outlier or magnetic disturbance)
//   }
//
// Expected Output:
//   - Yaw drift corrected (heading aligned with magnetic north)
//   - Orientation covariance reduced
//   - Outliers/disturbances rejected automatically

#pragma once

#include "ekf_state.hpp"
#include "ekf_update.hpp"
#include "core/types.hpp"

#include <Eigen/Dense>

namespace fusion {

/**
 * @brief World Magnetic Model (WMM2025) simplified reference
 *
 * Provides magnetic field vector in navigation frame (NED)
 * based on WGS84 position (latitude, longitude, altitude)
 *
 * Full WMM uses spherical harmonics (complex). This is a simplified
 * model with representative values for testing/prototyping.
 *
 * For production, integrate full NOAA WMM2025 model:
 * https://www.ncei.noaa.gov/products/world-magnetic-model
 */
class WorldMagneticModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Set observer location for magnetic field calculation
     *
     * @param latitude_deg Latitude in degrees [-90, 90]
     * @param longitude_deg Longitude in degrees [-180, 180]
     * @param altitude_m Altitude above WGS84 ellipsoid [m]
     */
    void set_location(double latitude_deg, double longitude_deg, double altitude_m);

    /**
     * @brief Get reference magnetic field vector in NED frame
     *
     * Returns the expected magnetic field vector at the current location
     * in the local NED (North-East-Down) navigation frame.
     *
     * @return Magnetic field vector [North, East, Down] in μT (micro-Tesla)
     */
    Vector3d get_reference_field_ned() const;

    /**
     * @brief Get magnetic declination (angle between true north and magnetic north)
     *
     * @return Declination in radians (positive = magnetic north is east of true north)
     */
    double get_declination() const { return declination_rad_; }

    /**
     * @brief Get magnetic inclination (dip angle)
     *
     * @return Inclination in radians (positive = field points downward)
     */
    double get_inclination() const { return inclination_rad_; }

    /**
     * @brief Get total field strength
     *
     * @return Field strength in μT
     */
    double get_field_strength() const { return field_strength_; }

private:
    double latitude_deg_;
    double longitude_deg_;
    double altitude_m_;

    // Magnetic field parameters (computed from location)
    double declination_rad_;   ///< Declination (angle from true north)
    double inclination_rad_;   ///< Inclination (dip angle)
    double field_strength_;    ///< Total field strength [μT]

    /**
     * @brief Compute magnetic field parameters from location
     *
     * Simplified model using lookup table / analytical approximation
     */
    void compute_magnetic_field();
};

/**
 * @brief Magnetometer measurement update
 *
 * Provides heading correction using 3-axis magnetometer measurements
 * with adaptive noise rejection and World Magnetic Model reference
 */
class MagUpdate {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor
     *
     * @param nominal_noise_std Nominal magnetometer noise std dev [μT] (default: 0.5 μT)
     * @param adaptive_noise_factor Factor for adaptive noise scaling (default: 5.0)
     */
    MagUpdate(double nominal_noise_std = 0.5, double adaptive_noise_factor = 5.0);

    /**
     * @brief Set observer location for WMM
     *
     * Must be called before first update to initialize reference field
     *
     * @param latitude_deg Latitude in degrees [-90, 90]
     * @param longitude_deg Longitude in degrees [-180, 180]
     * @param altitude_m Altitude above WGS84 ellipsoid [m]
     */
    void set_location(double latitude_deg, double longitude_deg, double altitude_m);

    /**
     * @brief Update EKF state with magnetometer measurement
     *
     * Performs full measurement update:
     * 1. Get reference field from WMM
     * 2. Predict measurement h(x) = R_nb^T * m_n
     * 3. Compute innovation y = z - h(x)
     * 4. Compute Jacobian H
     * 5. Adaptive noise (increase R if magnetic disturbance)
     * 6. Generic update with chi-square gating
     *
     * @param state EKF state to update (modified in-place)
     * @param mag_body_measured Measured magnetic field in body frame [μT]
     * @param enable_gating If true, reject outliers via chi-square test (default: true)
     * @return true if update accepted, false if rejected as outlier/disturbance
     */
    bool update(
        EkfState& state,
        const Vector3d& mag_body_measured,
        bool enable_gating = true);

    /**
     * @brief Check if magnetic disturbance is detected
     *
     * Compares measured field strength to reference field strength.
     * If deviation exceeds threshold, likely magnetic disturbance.
     *
     * @param mag_body_measured Measured magnetic field in body frame [μT]
     * @return true if disturbance detected
     */
    bool is_magnetic_disturbance(const Vector3d& mag_body_measured) const;

    /**
     * @brief Get reference magnetic field in NED frame
     *
     * @return Reference field from WMM [μT]
     */
    Vector3d get_reference_field() const {
        return wmm_.get_reference_field_ned();
    }

    /**
     * @brief Get last innovation (for diagnostics)
     *
     * @return Last innovation vector [μT]
     */
    Vector3d get_last_innovation() const { return last_innovation_; }

    /**
     * @brief Get last adaptive noise factor (for diagnostics)
     *
     * @return Last adaptive noise scaling factor (1.0 = nominal, >1.0 = increased noise)
     */
    double get_last_adaptive_factor() const { return last_adaptive_factor_; }

private:
    WorldMagneticModel wmm_;          ///< World Magnetic Model
    MeasurementUpdate update_;        ///< Generic measurement update

    double nominal_noise_std_;        ///< Nominal magnetometer noise [μT]
    double adaptive_noise_factor_;    ///< Scaling factor for adaptive noise

    // Diagnostics (for monitoring)
    Vector3d last_innovation_;        ///< Last innovation vector
    double last_adaptive_factor_;     ///< Last adaptive noise factor

    /**
     * @brief Compute adaptive measurement noise covariance
     *
     * Increases noise if measured field strength deviates from reference
     * (indicates magnetic disturbance)
     *
     * @param mag_body_measured Measured magnetic field in body frame [μT]
     * @return Adaptive noise covariance R (3×3)
     */
    Matrix3d compute_adaptive_noise(const Vector3d& mag_body_measured) const;

    /**
     * @brief Compute measurement Jacobian
     *
     * H = ∂h/∂δx where h(x) = R_nb^T * m_n
     *
     * For magnetometer (only depends on orientation):
     * ∂h/∂δθ = [R_nb^T * m_n]_× = [predicted_mag_body]_×
     * ∂h/∂(other states) = 0
     *
     * @param predicted_mag_body Predicted magnetic field in body frame
     * @return Jacobian H (3×15)
     */
    Eigen::Matrix<double, 3, 15> compute_jacobian(const Vector3d& predicted_mag_body) const;
};

}  // namespace fusion
