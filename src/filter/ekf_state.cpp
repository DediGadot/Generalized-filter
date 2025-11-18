// Error-State EKF Implementation
#include "filter/ekf_state.hpp"
#include "math/vector_math.hpp"

#include <cmath>

namespace fusion {

EkfState::EkfState() {
    // Default initialization: identity quaternion, zero position/velocity/biases
    nominal_ = NominalState();
    error_ = ErrorState();
}

void EkfState::initialize(
    const Vector3d& position,
    const Vector3d& velocity,
    const Quaterniond& attitude,
    int64_t timestamp_ns) {

    nominal_.q_nb = attitude.normalized();
    nominal_.v_n = velocity;
    nominal_.p_n = position;
    nominal_.b_g.setZero();
    nominal_.b_a.setZero();
    nominal_.timestamp_ns = timestamp_ns;

    // Error state starts at zero
    error_.dx.setZero();
}

void EkfState::set_initial_covariance(
    double sigma_theta,
    double sigma_v,
    double sigma_p,
    double sigma_bg,
    double sigma_ba) {

    error_.set_initial_covariance(sigma_theta, sigma_v, sigma_p, sigma_bg, sigma_ba);
}

void EkfState::predict(
    const PreintegratedImu& preint,
    const Vector3d& gravity) {

    // ============================================================
    // NOMINAL STATE PROPAGATION (Full Non-Linear Kinematics)
    // ============================================================

    // Rotation in navigation frame from body frame at start
    Matrix3d R_n_b = nominal_.q_nb.toRotationMatrix();

    // Propagate position (p_k+1 = p_k + v_k*dt + 0.5*g*dt² + R_k*Δp)
    double dt = preint.dt;
    nominal_.p_n = nominal_.p_n
                 + nominal_.v_n * dt
                 + 0.5 * gravity * dt * dt
                 + R_n_b * preint.delta_p;

    // Propagate velocity (v_k+1 = v_k + g*dt + R_k*Δv)
    nominal_.v_n = nominal_.v_n
                 + gravity * dt
                 + R_n_b * preint.delta_v;

    // Propagate orientation (q_k+1 = q_k ⊗ ΔR)
    nominal_.q_nb = (nominal_.q_nb * preint.delta_R).normalized();

    // Biases evolve slowly (random walk model, no deterministic update)
    // They are corrected only via measurement updates
    // nominal_.b_g remains unchanged (updated via measurements)
    // nominal_.b_a remains unchanged (updated via measurements)

    // Update timestamp
    nominal_.timestamp_ns = preint.end_time_ns;

    // ============================================================
    // ERROR STATE COVARIANCE PROPAGATION (Linearized Dynamics)
    // ============================================================

    // Compute state transition matrix F and noise Jacobian G
    Eigen::Matrix<double, 15, 15> F = compute_F(preint, gravity);
    Eigen::Matrix<double, 15, 12> G = compute_G(preint);

    // Discrete-time state transition: Φ = I + F*dt
    // (First-order approximation, sufficient for small dt)
    Eigen::Matrix<double, 15, 15> Phi = Eigen::Matrix<double, 15, 15>::Identity() + F * dt;

    // Process noise covariance Q (12×12)
    // Q = diag(σ_g², σ_a², σ_bg², σ_ba²) * dt
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();

    // Get noise parameters from preintegration
    double sigma_g = preint.noise_params.gyro_noise_density;
    double sigma_a = preint.noise_params.accel_noise_density;
    double sigma_bg = preint.noise_params.gyro_random_walk;
    double sigma_ba = preint.noise_params.accel_random_walk;

    // Gyro noise (affects rotation error)
    Q.block<3, 3>(0, 0) = Matrix3d::Identity() * (sigma_g * sigma_g * dt);

    // Accel noise (affects velocity error)
    Q.block<3, 3>(3, 3) = Matrix3d::Identity() * (sigma_a * sigma_a * dt);

    // Gyro bias random walk
    Q.block<3, 3>(6, 6) = Matrix3d::Identity() * (sigma_bg * sigma_bg * dt);

    // Accel bias random walk
    Q.block<3, 3>(9, 9) = Matrix3d::Identity() * (sigma_ba * sigma_ba * dt);

    // Covariance propagation: P = Φ*P*Φ^T + G*Q*G^T
    // Use .noalias() to avoid creating temporaries (Eigen optimization)
    Eigen::Matrix<double, 15, 15> P_new;
    P_new.noalias() = Phi * error_.P * Phi.transpose() + G * Q * G.transpose();

    // Ensure symmetry (numerical errors can break it)
    error_.P = 0.5 * (P_new + P_new.transpose());

    // Error state remains zero (it only grows via covariance, gets values during update)
    error_.dx.setZero();
}

void EkfState::inject_error(const Eigen::Matrix<double, 15, 1>& error_update) {
    // Extract error components
    Vector3d delta_theta = error_update.segment<3>(ErrorState::DTHETA);
    Vector3d delta_v = error_update.segment<3>(ErrorState::DV);
    Vector3d delta_p = error_update.segment<3>(ErrorState::DP);
    Vector3d delta_bg = error_update.segment<3>(ErrorState::DBG);
    Vector3d delta_ba = error_update.segment<3>(ErrorState::DBA);

    // Inject rotation error: q ← q ⊗ Exp(δθ)
    Quaterniond delta_q = quaternion_exp(delta_theta);
    nominal_.q_nb = (nominal_.q_nb * delta_q).normalized();

    // Inject velocity error: v ← v + δv
    nominal_.v_n += delta_v;

    // Inject position error: p ← p + δp
    nominal_.p_n += delta_p;

    // Inject bias errors: b ← b + δb
    nominal_.b_g += delta_bg;
    nominal_.b_a += delta_ba;
}

void EkfState::reset_error() {
    // After injection, error state should be zero
    error_.dx.setZero();

    // Covariance remains (represents uncertainty in the updated nominal state)
}

Eigen::Matrix<double, 15, 15> EkfState::compute_F(
    const PreintegratedImu& preint,
    const Vector3d& gravity) const {

    // State transition matrix F (15×15)
    // Describes continuous-time error dynamics: δẋ = F*δx + G*n
    //
    // Error state: [δθ(3), δv(3), δp(3), δb_g(3), δb_a(3)]
    //
    // F has the structure:
    //     δθ   δv   δp  δb_g  δb_a
    // δθ [  0    0    0   -I     0  ]
    // δv [ -R[a]× 0    0    0   -R  ]
    // δp [  0    I    0    0     0  ]
    // δb_g[ 0    0    0    0     0  ]
    // δb_a[ 0    0    0    0     0  ]
    //
    // where:
    // - R = rotation matrix from body to navigation frame
    // - [a]× = skew-symmetric matrix of specific force (accelerometer - bias)
    // - I = 3×3 identity

    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();

    // Get rotation matrix
    Matrix3d R_n_b = nominal_.q_nb.toRotationMatrix();

    // Get specific force (average accelerometer reading in preintegration)
    // This is approximated from the velocity change
    Vector3d specific_force = Vector3d::Zero();
    if (preint.dt > 1e-6) {
        // Approximate: a ≈ Δv / dt (in body frame)
        // We don't have direct access to raw IMU, so use preintegrated Δv
        specific_force = preint.delta_v / preint.dt;
    }

    // ∂δθ/∂δb_g = -I (gyro bias affects rotation error)
    F.block<3, 3>(ErrorState::DTHETA, ErrorState::DBG) = -Matrix3d::Identity();

    // ∂δv/∂δθ = -R*[a]× (rotation error affects velocity via gravity direction)
    Matrix3d a_skew = skew_symmetric(specific_force);
    F.block<3, 3>(ErrorState::DV, ErrorState::DTHETA) = -R_n_b * a_skew;

    // ∂δv/∂δb_a = -R (accel bias directly affects velocity)
    F.block<3, 3>(ErrorState::DV, ErrorState::DBA) = -R_n_b;

    // ∂δp/∂δv = I (velocity error integrates to position error)
    F.block<3, 3>(ErrorState::DP, ErrorState::DV) = Matrix3d::Identity();

    // Biases evolve via random walk (no coupling with other states)
    // ∂δb_g/∂* = 0, ∂δb_a/∂* = 0

    return F;
}

Eigen::Matrix<double, 15, 12> EkfState::compute_G(
    const PreintegratedImu& preint) const {

    // Noise Jacobian G (15×12)
    // Maps process noise to error state: δẋ = F*δx + G*n
    //
    // Process noise: n = [η_g(3), η_a(3), η_bg(3), η_ba(3)]
    // where:
    // - η_g  = gyro noise (white noise)
    // - η_a  = accel noise (white noise)
    // - η_bg = gyro bias random walk
    // - η_ba = accel bias random walk
    //
    // G has the structure:
    //     η_g  η_a  η_bg  η_ba
    // δθ [ -I    0    0     0  ]
    // δv [  0   -R    0     0  ]
    // δp [  0    0    0     0  ]
    // δb_g[ 0    0    I     0  ]
    // δb_a[ 0    0    0     I  ]
    //
    // Note: Noise affects states directly or via integration

    Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();

    // Get rotation matrix
    Matrix3d R_n_b = nominal_.q_nb.toRotationMatrix();

    // ∂δθ/∂η_g = -I (gyro noise directly affects rotation error)
    G.block<3, 3>(ErrorState::DTHETA, 0) = -Matrix3d::Identity();

    // ∂δv/∂η_a = -R (accel noise affects velocity, rotated to nav frame)
    G.block<3, 3>(ErrorState::DV, 3) = -R_n_b;

    // ∂δb_g/∂η_bg = I (gyro bias driven by random walk)
    G.block<3, 3>(ErrorState::DBG, 6) = Matrix3d::Identity();

    // ∂δb_a/∂η_ba = I (accel bias driven by random walk)
    G.block<3, 3>(ErrorState::DBA, 9) = Matrix3d::Identity();

    return G;
}

}  // namespace fusion
