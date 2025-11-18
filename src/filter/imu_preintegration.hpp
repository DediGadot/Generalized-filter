// IMU Preintegration using Forster et al. Algorithm
//
// Purpose: Efficiently integrate IMU measurements between EKF epochs
// Reference: "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
//            Forster et al., IEEE TRO 2017
//            https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf
//
// Key Features:
// - Delta formulation: independent of absolute state
// - Bias correction via Jacobians (no re-integration needed)
// - Midpoint integration for numerical accuracy
// - Covariance propagation for uncertainty quantification
//
// Sample Usage:
//   ImuPreintegration preint(bias_gyro, bias_accel);
//   for (auto& sample : imu_samples) {
//       preint.integrate(sample);
//   }
//   auto result = preint.get_result();
//
// Expected Output:
//   - ΔR, Δv, Δp: Rotation, velocity, position deltas
//   - Jacobians: ∂Δ/∂b_g, ∂Δ/∂b_a for bias correction
//   - Covariance: 9×9 uncertainty of delta measurement
//
// Performance Target: <10µs per integration step on Snapdragon AR1 Gen 1

#pragma once

#include "core/quaternion.hpp"
#include "core/sensor_types.hpp"
#include "core/types.hpp"

#include <Eigen/Dense>
#include <vector>

namespace fusion {

/**
 * @brief IMU Preintegration accumulator
 *
 * Accumulates IMU measurements between times i and j into delta measurements.
 * Delta measurements are in the frame at time i (local frame).
 *
 * Math:
 *   ΔR_{ij} = ∏ Exp((ω_m - b_g) dt)      # Rotation delta
 *   Δv_{ij} = ∫ ΔR(t) (a_m - b_a) dt     # Velocity delta
 *   Δp_{ij} = ∫∫ ΔR(t) (a_m - b_a) dt    # Position delta
 *
 * These deltas are independent of the global state and can be corrected
 * for bias changes via first-order Jacobians.
 */
class ImuPreintegration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor
     *
     * @param bias_gyro Initial gyroscope bias [rad/s], body frame
     * @param bias_accel Initial accelerometer bias [m/s²], body frame
     * @param noise_params IMU noise parameters (for covariance)
     */
    ImuPreintegration(
        const Vector3d& bias_gyro = Vector3d::Zero(),
        const Vector3d& bias_accel = Vector3d::Zero(),
        const ImuNoiseParams& noise_params = ImuNoiseParams::default_consumer());

    /**
     * @brief Integrate a single IMU sample
     *
     * Uses midpoint integration for better numerical accuracy.
     *
     * @param sample IMU measurement (gyro + accel + timestamp)
     */
    void integrate(const ImuSample& sample);

    /**
     * @brief Reset preintegration to initial state
     *
     * Call this when starting a new preintegration interval.
     *
     * @param bias_gyro New gyroscope bias
     * @param bias_accel New accelerometer bias
     */
    void reset(const Vector3d& bias_gyro = Vector3d::Zero(),
               const Vector3d& bias_accel = Vector3d::Zero());

    /**
     * @brief Get preintegrated result
     *
     * @return PreintegratedImu struct with deltas, Jacobians, and covariance
     */
    PreintegratedImu get_result() const;

    /**
     * @brief Update bias and correct deltas using Jacobians
     *
     * This is the key advantage of preintegration: we can correct for bias
     * changes without re-integrating all IMU samples.
     *
     * @param new_bias_gyro Updated gyroscope bias
     * @param new_bias_accel Updated accelerometer bias
     */
    void update_bias(const Vector3d& new_bias_gyro,
                     const Vector3d& new_bias_accel);

    // Getters
    const Quaterniond& delta_R() const { return delta_R_; }
    const Vector3d& delta_v() const { return delta_v_; }
    const Vector3d& delta_p() const { return delta_p_; }
    double delta_t() const { return delta_t_; }
    int num_samples() const { return num_samples_; }

private:
    // Update Jacobians (∂Δ/∂b)
    void update_jacobians(const Vector3d& omega_corrected,
                          const Vector3d& accel_corrected,
                          double dt);

    // Update covariance
    void update_covariance(double dt);

    // Skew-symmetric matrix (for Jacobian computation)
    static Matrix3d skew(const Vector3d& v);

    // === State ===
    Quaterniond delta_R_;       ///< Rotation delta (relative to frame at start)
    Vector3d delta_v_;          ///< Velocity delta [m/s]
    Vector3d delta_p_;          ///< Position delta [m]

    // Biases
    Vector3d bias_gyro_;        ///< Gyroscope bias [rad/s]
    Vector3d bias_accel_;       ///< Accelerometer bias [m/s²]

    // Jacobians (for bias correction)
    Matrix3d dR_dbg_;           ///< ∂ΔR/∂b_gyro
    Matrix3d dv_dbg_;           ///< ∂Δv/∂b_gyro
    Matrix3d dv_dba_;           ///< ∂Δv/∂b_accel
    Matrix3d dp_dbg_;           ///< ∂Δp/∂b_gyro
    Matrix3d dp_dba_;           ///< ∂Δp/∂b_accel

    // Covariance (9×9: [ΔR Δv Δp])
    Eigen::Matrix<double, 9, 9> covariance_;

    // Noise parameters
    ImuNoiseParams noise_params_;

    // Timing
    int64_t start_time_ns_;     ///< Start timestamp
    int64_t end_time_ns_;       ///< End timestamp
    double delta_t_;            ///< Total integration time [s]

    // Previous sample (for midpoint integration)
    ImuSample prev_sample_;
    bool has_prev_sample_;

    // Statistics
    int num_samples_;
};

}  // namespace fusion
