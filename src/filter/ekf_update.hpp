// Generic EKF Measurement Update
//
// Purpose: Provides a generic framework for updating the error-state EKF with any measurement
// Reference: "Kalman Filtering: Theory and Practice Using MATLAB" - Grewal & Andrews
//
// Key Features:
// - Generic measurement update (works with any measurement type)
// - Joseph form covariance update (numerically stable, guaranteed positive semi-definite)
// - Chi-square gating for outlier rejection (Mahalanobis distance test)
// - Measurement residual (innovation) analysis
//
// Measurement Model:
//   z = h(x) + v
//   where:
//     z = measurement vector (size m)
//     h(x) = measurement function (non-linear)
//     v ~ N(0, R) = measurement noise
//
// Linearized Measurement Model:
//   δz = H * δx + v
//   where:
//     H = ∂h/∂x = measurement Jacobian (m × 15)
//     δz = z_measured - h(x_nominal) = innovation
//
// Update Equations:
//   1. Innovation: y = z - h(x_nominal)
//   2. Innovation covariance: S = H*P*H^T + R
//   3. Kalman gain: K = P*H^T * S^-1
//   4. Error state update: δx = K * y
//   5. Joseph form covariance: P = (I-KH)*P*(I-KH)^T + K*R*K^T
//
// Sample Usage:
//   MeasurementUpdate update;
//
//   // Set measurement Jacobian H and noise covariance R
//   update.set_measurement_model(H, R);
//
//   // Compute innovation (measurement - prediction)
//   Vector innovation = measurement - predicted_measurement;
//
//   // Perform update (with outlier rejection)
//   if (update.update(state, innovation)) {
//       // Update accepted
//       state.inject_error(state.error_state());
//       state.reset_error();
//   } else {
//       // Update rejected (outlier)
//   }
//
// Expected Output:
//   - Error state updated with measurement information
//   - Covariance reduced (uncertainty decreases)
//   - Outliers rejected via chi-square test

#pragma once

#include "ekf_state.hpp"
#include "core/types.hpp"

#include <Eigen/Dense>

namespace fusion {

/**
 * @brief Generic EKF measurement update
 *
 * Provides a reusable framework for updating the error-state EKF
 * with any type of measurement (GNSS, magnetometer, vision, etc.)
 */
class MeasurementUpdate {
public:
    /**
     * @brief Constructor
     */
    MeasurementUpdate();

    /**
     * @brief Update error state with measurement
     *
     * Performs the full EKF measurement update:
     * 1. Innovation covariance S = H*P*H^T + R
     * 2. Kalman gain K = P*H^T * S^-1
     * 3. Error state update δx = K * y
     * 4. Joseph form covariance update
     * 5. Chi-square gating (optional)
     *
     * @param state EKF state to update (modified in-place)
     * @param innovation Measurement innovation (z - h(x_nominal))
     * @param H Measurement Jacobian (m × 15)
     * @param R Measurement noise covariance (m × m)
     * @param enable_gating If true, reject outliers via chi-square test
     * @param chi_square_threshold Threshold for outlier rejection (default: 7.815 for 95% confidence, 3 DOF)
     * @return true if update accepted, false if rejected as outlier
     */
    template <int MeasurementDim>
    bool update(
        EkfState& state,
        const Eigen::Matrix<double, MeasurementDim, 1>& innovation,
        const Eigen::Matrix<double, MeasurementDim, 15>& H,
        const Eigen::Matrix<double, MeasurementDim, MeasurementDim>& R,
        bool enable_gating = true,
        double chi_square_threshold = 7.815) {

        // Get current error covariance
        const auto& P = state.error().P;

        // === 1. Innovation Covariance ===
        // S = H*P*H^T + R
        Eigen::Matrix<double, MeasurementDim, MeasurementDim> S;
        S.noalias() = H * P * H.transpose() + R;

        // === 2. Chi-Square Gating (Mahalanobis Distance Test) ===
        if (enable_gating) {
            // Mahalanobis distance: d² = y^T * S^-1 * y
            double mahalanobis_sq = innovation.transpose() * S.inverse() * innovation;

            // If distance exceeds threshold, reject as outlier
            if (mahalanobis_sq > chi_square_threshold) {
                // Outlier detected - reject measurement
                return false;
            }
        }

        // === 3. Kalman Gain ===
        // K = P*H^T * S^-1
        Eigen::Matrix<double, 15, MeasurementDim> K;
        K.noalias() = P * H.transpose() * S.inverse();

        // === 4. Error State Update ===
        // δx = K * y
        Eigen::Matrix<double, 15, 1> delta_x;
        delta_x.noalias() = K * innovation;

        // Update error state
        state.error().dx = delta_x;

        // === 5. Joseph Form Covariance Update ===
        // P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
        //
        // Why Joseph form?
        // - Numerically stable (guaranteed positive semi-definite)
        // - Standard form P = (I-K*H)*P can lose symmetry/positive-definiteness
        // - Joseph form adds the K*R*K^T term which compensates for numerical errors
        //
        // This is MANDATORY for production systems!

        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        Eigen::Matrix<double, 15, 15> P_new;

        // First term: (I-KH)*P*(I-KH)^T
        P_new.noalias() = I_KH * P * I_KH.transpose();

        // Second term: K*R*K^T
        P_new.noalias() += K * R * K.transpose();

        // Enforce symmetry (numerical errors can break it)
        state.error().P = 0.5 * (P_new + P_new.transpose());

        // Update accepted
        return true;
    }

    /**
     * @brief Compute Mahalanobis distance (for outlier detection)
     *
     * The Mahalanobis distance measures how many standard deviations
     * the innovation is from the predicted measurement.
     *
     * d² = y^T * S^-1 * y
     *
     * For a 3D measurement (e.g., position), chi-square thresholds:
     * - 95% confidence: 7.815
     * - 99% confidence: 11.345
     *
     * @param innovation Measurement innovation
     * @param S Innovation covariance
     * @return Mahalanobis distance squared
     */
    template <int MeasurementDim>
    static double compute_mahalanobis_distance(
        const Eigen::Matrix<double, MeasurementDim, 1>& innovation,
        const Eigen::Matrix<double, MeasurementDim, MeasurementDim>& S) {

        return innovation.transpose() * S.inverse() * innovation;
    }

    /**
     * @brief Get chi-square threshold for given confidence level and DOF
     *
     * Common values:
     * - 1 DOF: 95% = 3.841, 99% = 6.635
     * - 2 DOF: 95% = 5.991, 99% = 9.210
     * - 3 DOF: 95% = 7.815, 99% = 11.345
     * - 4 DOF: 95% = 9.488, 99% = 13.277
     *
     * @param dof Degrees of freedom (measurement dimension)
     * @param confidence_level Confidence level (0.95 or 0.99)
     * @return Chi-square threshold
     */
    static double get_chi_square_threshold(int dof, double confidence_level = 0.95);

private:
    // Statistics (for debugging/monitoring)
    int num_updates_;
    int num_outliers_;
};

/**
 * @brief Measurement residual statistics
 *
 * Useful for monitoring filter health and detecting failures.
 */
struct ResidualStatistics {
    double mean;                ///< Mean of residual (should be ~0)
    double std_dev;             ///< Standard deviation of residual
    double mahalanobis_dist;    ///< Mahalanobis distance (normalized)
    bool is_outlier;            ///< True if rejected by chi-square test

    ResidualStatistics()
        : mean(0), std_dev(0), mahalanobis_dist(0), is_outlier(false) {}
};

}  // namespace fusion
