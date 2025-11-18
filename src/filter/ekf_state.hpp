// Error-State Extended Kalman Filter State Management
//
// Purpose: Manages the hybrid nominal + error state representation for INS/GNSS fusion
// Reference: "Quaternion kinematics for the error-state Kalman filter" - Joan Solà
//            https://arxiv.org/abs/1711.02508
//
// Key Concept: Error-State Formulation
// - Nominal state: Full non-linear state (quaternion, position, velocity, biases)
// - Error state: Small deviation from nominal (15D vector, always small)
// - Covariance: Uncertainty of error state (15×15 matrix)
//
// Why Error-State?
// 1. Error is always small → linearization is valid
// 2. Quaternion constraint handled in nominal state
// 3. Error state rotation is 3D (axis-angle), not 4D quaternion
// 4. After update, inject error into nominal and reset error to zero
//
// State Vector (15D):
//   δx = [δθ(3), δv(3), δp(3), δb_g(3), δb_a(3)]
//
// where:
//   δθ  = rotation error (axis-angle representation) [rad]
//   δv  = velocity error in navigation frame [m/s]
//   δp  = position error in navigation frame [m]
//   δb_g = gyroscope bias error [rad/s]
//   δb_a = accelerometer bias error [m/s²]
//
// Nominal State:
//   q_nb  = quaternion (body → navigation frame)
//   v_n   = velocity in navigation frame [m/s]
//   p_n   = position in navigation frame [m]
//   b_g   = gyroscope bias in body frame [rad/s]
//   b_a   = accelerometer bias in body frame [m/s²]
//
// Sample Usage:
//   EkfState state;
//   state.initialize(initial_position, initial_velocity, initial_attitude);
//
//   // Prediction step
//   state.predict(preintegrated_imu, dt);
//
//   // Update step (after measurement)
//   state.inject_error(error_state);  // error → nominal
//   state.reset_error();              // error = 0
//
// Expected Output:
//   - Nominal state: smoothly evolving pose/velocity/biases
//   - Error state: small deviations, reset to zero after each update
//   - Covariance: growing during prediction, shrinking during update

#pragma once

#include "core/quaternion.hpp"
#include "core/types.hpp"
#include "imu_preintegration.hpp"

#include <Eigen/Dense>

namespace fusion {

/**
 * @brief Nominal state (full non-linear state)
 *
 * This is the "best estimate" of the true state.
 * It evolves via non-linear dynamics (quaternion multiplication, etc.)
 */
struct NominalState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Quaterniond q_nb;      ///< Orientation: body → navigation frame
    Vector3d v_n;          ///< Velocity in navigation frame [m/s]
    Vector3d p_n;          ///< Position in navigation frame [m]
    Vector3d b_g;          ///< Gyroscope bias in body frame [rad/s]
    Vector3d b_a;          ///< Accelerometer bias in body frame [m/s²]

    int64_t timestamp_ns;  ///< State timestamp [nanoseconds]

    // Constructor
    NominalState()
        : q_nb(Quaterniond::Identity()),
          v_n(Vector3d::Zero()),
          p_n(Vector3d::Zero()),
          b_g(Vector3d::Zero()),
          b_a(Vector3d::Zero()),
          timestamp_ns(0) {}
};

/**
 * @brief Error state (small deviation from nominal, always linearized)
 *
 * This represents the uncertainty in the nominal state.
 * It's propagated via linearized dynamics (Jacobians).
 * After measurement update, it's injected into nominal and reset to zero.
 */
struct ErrorState {
    // Error state vector (15×1)
    // [δθ(3), δv(3), δp(3), δb_g(3), δb_a(3)]
    Eigen::Matrix<double, 15, 1> dx;

    // Error state covariance (15×15)
    Eigen::Matrix<double, 15, 15> P;

    // Indices for error state vector
    enum Index {
        DTHETA = 0,  ///< Rotation error (axis-angle) [0:3]
        DV = 3,      ///< Velocity error [3:6]
        DP = 6,      ///< Position error [6:9]
        DBG = 9,     ///< Gyro bias error [9:12]
        DBA = 12     ///< Accel bias error [12:15]
    };

    // Constructor: zero error, initial covariance
    ErrorState() {
        dx.setZero();
        P.setZero();
    }

    // Set initial covariance (diagonal)
    void set_initial_covariance(
        double sigma_theta,  ///< Initial rotation uncertainty [rad]
        double sigma_v,      ///< Initial velocity uncertainty [m/s]
        double sigma_p,      ///< Initial position uncertainty [m]
        double sigma_bg,     ///< Initial gyro bias uncertainty [rad/s]
        double sigma_ba      ///< Initial accel bias uncertainty [m/s²]
    ) {
        P.setZero();

        // Rotation uncertainty
        P.block<3, 3>(DTHETA, DTHETA) = Matrix3d::Identity() * (sigma_theta * sigma_theta);

        // Velocity uncertainty
        P.block<3, 3>(DV, DV) = Matrix3d::Identity() * (sigma_v * sigma_v);

        // Position uncertainty
        P.block<3, 3>(DP, DP) = Matrix3d::Identity() * (sigma_p * sigma_p);

        // Gyro bias uncertainty
        P.block<3, 3>(DBG, DBG) = Matrix3d::Identity() * (sigma_bg * sigma_bg);

        // Accel bias uncertainty
        P.block<3, 3>(DBA, DBA) = Matrix3d::Identity() * (sigma_ba * sigma_ba);
    }
};

/**
 * @brief Complete EKF state (nominal + error)
 *
 * This class manages both the nominal state and error state,
 * providing functions for prediction, update, and error injection.
 */
class EkfState {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor
     */
    EkfState();

    /**
     * @brief Initialize state with known values
     *
     * @param position Initial position in navigation frame [m]
     * @param velocity Initial velocity in navigation frame [m/s]
     * @param attitude Initial attitude quaternion (body → nav)
     * @param timestamp_ns Initial timestamp [nanoseconds]
     */
    void initialize(
        const Vector3d& position,
        const Vector3d& velocity,
        const Quaterniond& attitude,
        int64_t timestamp_ns = 0);

    /**
     * @brief Set initial error state covariance
     *
     * @param sigma_theta Initial rotation uncertainty [rad]
     * @param sigma_v Initial velocity uncertainty [m/s]
     * @param sigma_p Initial position uncertainty [m]
     * @param sigma_bg Initial gyro bias uncertainty [rad/s]
     * @param sigma_ba Initial accel bias uncertainty [m/s²]
     */
    void set_initial_covariance(
        double sigma_theta,
        double sigma_v,
        double sigma_p,
        double sigma_bg,
        double sigma_ba);

    /**
     * @brief Predict state using IMU preintegration
     *
     * This propagates the nominal state using the full non-linear dynamics.
     * The error state covariance is propagated using linearized dynamics.
     *
     * @param preint Preintegrated IMU measurements
     * @param gravity Gravity vector in navigation frame [m/s²]
     */
    void predict(
        const PreintegratedImu& preint,
        const Vector3d& gravity = Vector3d(0, 0, 9.81));

    /**
     * @brief Inject error state into nominal state
     *
     * After measurement update, the error state contains corrections.
     * This function applies those corrections to the nominal state:
     * - q_nb ← q_nb ⊗ Exp(δθ)
     * - v_n ← v_n + δv
     * - p_n ← p_n + δp
     * - b_g ← b_g + δb_g
     * - b_a ← b_a + δb_a
     *
     * @param error_update The error state correction to apply
     */
    void inject_error(const Eigen::Matrix<double, 15, 1>& error_update);

    /**
     * @brief Reset error state to zero after injection
     *
     * After injecting error into nominal, the error state should be zero.
     * The covariance remains unchanged (it represents uncertainty in the
     * updated nominal state).
     */
    void reset_error();

    // Getters
    const NominalState& nominal() const { return nominal_; }
    const ErrorState& error() const { return error_; }
    NominalState& nominal() { return nominal_; }
    ErrorState& error() { return error_; }

    // Get individual state components
    const Quaterniond& attitude() const { return nominal_.q_nb; }
    const Vector3d& velocity() const { return nominal_.v_n; }
    const Vector3d& position() const { return nominal_.p_n; }
    const Vector3d& gyro_bias() const { return nominal_.b_g; }
    const Vector3d& accel_bias() const { return nominal_.b_a; }

    const Eigen::Matrix<double, 15, 1>& error_state() const { return error_.dx; }
    const Eigen::Matrix<double, 15, 15>& covariance() const { return error_.P; }

private:
    NominalState nominal_;  ///< Nominal state (non-linear)
    ErrorState error_;      ///< Error state (linear)

    /**
     * @brief Compute state transition matrix F (15×15)
     *
     * F describes how the error state evolves:
     * δx_dot = F * δx + G * n
     *
     * For discrete time: δx_k+1 ≈ (I + F*dt) * δx_k
     * Or more accurately: δx_k+1 = Φ * δx_k where Φ = exp(F*dt)
     *
     * @param preint Preintegrated IMU measurements
     * @param gravity Gravity vector in navigation frame
     * @return State transition matrix F
     */
    Eigen::Matrix<double, 15, 15> compute_F(
        const PreintegratedImu& preint,
        const Vector3d& gravity) const;

    /**
     * @brief Compute noise Jacobian G (15×12)
     *
     * G maps process noise to error state:
     * noise vector n = [η_g(3), η_a(3), η_bg(3), η_ba(3)]
     *
     * @param preint Preintegrated IMU measurements
     * @return Noise Jacobian G
     */
    Eigen::Matrix<double, 15, 12> compute_G(
        const PreintegratedImu& preint) const;
};

}  // namespace fusion
