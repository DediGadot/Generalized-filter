// IMU Preintegration Implementation
#include "filter/imu_preintegration.hpp"
#include "math/vector_math.hpp"

#include <cmath>

namespace fusion {

ImuPreintegration::ImuPreintegration(
    const Vector3d& bias_gyro,
    const Vector3d& bias_accel,
    const ImuNoiseParams& noise_params)
    : delta_R_(Quaterniond::Identity()),
      delta_v_(Vector3d::Zero()),
      delta_p_(Vector3d::Zero()),
      bias_gyro_(bias_gyro),
      bias_accel_(bias_accel),
      dR_dbg_(Matrix3d::Zero()),  // Zero initially: no accumulated effect at t=0
      dv_dbg_(Matrix3d::Zero()),
      dv_dba_(Matrix3d::Zero()),  // Zero initially
      dp_dbg_(Matrix3d::Zero()),
      dp_dba_(Matrix3d::Zero()),  // Zero initially
      covariance_(Eigen::Matrix<double, 9, 9>::Zero()),
      noise_params_(noise_params),
      start_time_ns_(0),
      end_time_ns_(0),
      delta_t_(0.0),
      has_prev_sample_(false),
      num_samples_(0) {}

void ImuPreintegration::reset(const Vector3d& bias_gyro,
                               const Vector3d& bias_accel) {
    delta_R_ = Quaterniond::Identity();
    delta_v_.setZero();
    delta_p_.setZero();

    bias_gyro_ = bias_gyro;
    bias_accel_ = bias_accel;

    dR_dbg_.setZero();  // Zero at t=0
    dv_dbg_.setZero();
    dv_dba_.setZero();  // Zero at t=0
    dp_dbg_.setZero();
    dp_dba_.setZero();  // Zero at t=0

    covariance_.setZero();

    start_time_ns_ = 0;
    end_time_ns_ = 0;
    delta_t_ = 0.0;

    has_prev_sample_ = false;
    num_samples_ = 0;
}

void ImuPreintegration::integrate(const ImuSample& sample) {
    // First sample: just store it
    if (!has_prev_sample_) {
        prev_sample_ = sample;
        has_prev_sample_ = true;
        start_time_ns_ = sample.timestamp_ns;
        end_time_ns_ = sample.timestamp_ns;
        return;
    }

    // Compute dt
    double dt = (sample.timestamp_ns - prev_sample_.timestamp_ns) * 1e-9;
    if (dt <= 0.0 || dt > 0.1) {  // Sanity check: 0 < dt < 100ms
        // Invalid dt - skip this sample
        prev_sample_ = sample;
        return;
    }

    // Bias-corrected measurements (previous sample)
    Vector3d omega_0 = prev_sample_.gyro.cast<double>() - bias_gyro_;
    Vector3d accel_0 = prev_sample_.accel.cast<double>() - bias_accel_;

    // Bias-corrected measurements (current sample)
    Vector3d omega_1 = sample.gyro.cast<double>() - bias_gyro_;
    Vector3d accel_1 = sample.accel.cast<double>() - bias_accel_;

    // Midpoint integration (more accurate than Euler)
    Vector3d omega_mid = 0.5 * (omega_0 + omega_1);
    Vector3d accel_mid = 0.5 * (accel_0 + accel_1);

    // === UPDATE ROTATION ===
    // ΔR_{k+1} = ΔR_k * Exp(ω_mid * dt)
    Vector3d theta = omega_mid * dt;  // Rotation vector
    Quaterniond dR = quaternion_exp(theta);
    Quaterniond delta_R_prev = delta_R_;
    delta_R_ = delta_R_ * dR;
    delta_R_.normalize();  // Prevent drift

    // === UPDATE VELOCITY ===
    // Δv_{k+1} = Δv_k + ΔR_k * a_mid * dt
    Matrix3d R_mat = delta_R_prev.toRotationMatrix();
    Vector3d a_rotated = R_mat * accel_mid;
    Vector3d delta_v_prev = delta_v_;  // Save for position update
    delta_v_ += a_rotated * dt;

    // === UPDATE POSITION ===
    // Δp_{k+1} = Δp_k + Δv_k * dt + 0.5 * ΔR_k * a_mid * dt²
    // IMPORTANT: Use delta_v_prev (value at step k), not the updated delta_v_
    delta_p_ += delta_v_prev * dt + 0.5 * a_rotated * dt * dt;

    // === UPDATE JACOBIANS ===
    update_jacobians(omega_mid, accel_mid, dt);

    // === UPDATE COVARIANCE ===
    update_covariance(dt);

    // Update timing
    end_time_ns_ = sample.timestamp_ns;
    delta_t_ += dt;

    // Store for next iteration
    prev_sample_ = sample;
    num_samples_++;
}

void ImuPreintegration::update_jacobians(const Vector3d& omega_corrected,
                                         const Vector3d& accel_corrected,
                                         double dt) {
    // Rotation matrix at current state (before update)
    Matrix3d R = delta_R_.toRotationMatrix();

    // Skew-symmetric matrices
    Matrix3d accel_skew = skew(accel_corrected);

    // Save old Jacobians (needed for subsequent updates)
    Matrix3d dR_dbg_old = dR_dbg_;
    Matrix3d dv_dbg_old = dv_dbg_;
    Matrix3d dv_dba_old = dv_dba_;

    // === Jacobian: ∂ΔR/∂b_gyro ===
    // From Forster et al. Eq. (45)
    // dR/dbg_{k+1} = dR/dbg_k - ΔR_k * Jr(θ) * dt
    Vector3d theta_vec = omega_corrected * dt;
    Matrix3d Jr = right_jacobian(theta_vec);
    dR_dbg_ = dR_dbg_old - R * Jr * dt;

    // === Jacobian: ∂Δv/∂b_gyro ===
    // From Forster et al. Eq. (46)
    // dv/dbg_{k+1} = dv/dbg_k - ΔR_k * [a_mid]× * dR/dbg_k * dt
    // IMPORTANT: Use dR_dbg_old (value at step k), not the updated value
    dv_dbg_ = dv_dbg_old - R * accel_skew * dR_dbg_old * dt;

    // === Jacobian: ∂Δv/∂b_accel ===
    // From Forster et al. Eq. (47)
    // dv/dba_{k+1} = dv/dba_k - ΔR_k * dt
    dv_dba_ = dv_dba_old - R * dt;

    // === Jacobian: ∂Δp/∂b_gyro ===
    // From Forster et al. Eq. (48)
    // dp/dbg_{k+1} = dp/dbg_k + dv/dbg_k * dt - 0.5 * ΔR_k * [a_mid]× * dR/dbg_k * dt²
    // IMPORTANT: Use old values (step k), not updated values
    dp_dbg_ = dp_dbg_ + dv_dbg_old * dt - 0.5 * R * accel_skew * dR_dbg_old * dt * dt;

    // === Jacobian: ∂Δp/∂b_accel ===
    // From Forster et al. Eq. (49)
    // dp/dba_{k+1} = dp/dba_k + dv/dba_k * dt - 0.5 * ΔR_k * dt²
    // IMPORTANT: Use old value (step k)
    dp_dba_ = dp_dba_ + dv_dba_old * dt - 0.5 * R * dt * dt;
}

void ImuPreintegration::update_covariance(double dt) {
    // Noise covariance (continuous time)
    // Q_c = diag(σ_g², σ_a², σ_bg², σ_ba²)
    double sigma_g = noise_params_.gyro_noise_density;
    double sigma_a = noise_params_.accel_noise_density;
    double sigma_bg = noise_params_.gyro_random_walk;
    double sigma_ba = noise_params_.accel_random_walk;

    // Discretized noise covariance
    // Q_d = Q_c * dt (for white noise)
    // Q_d = Q_c * dt³/3 (for random walk, integrated twice)
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();

    // Gyro noise
    Q.block<3, 3>(0, 0) = Matrix3d::Identity() * (sigma_g * sigma_g * dt);
    // Accel noise
    Q.block<3, 3>(3, 3) = Matrix3d::Identity() * (sigma_a * sigma_a * dt);
    // Gyro bias random walk
    Q.block<3, 3>(6, 6) = Matrix3d::Identity() * (sigma_bg * sigma_bg * dt);
    // Accel bias random walk
    Q.block<3, 3>(9, 9) = Matrix3d::Identity() * (sigma_ba * sigma_ba * dt);

    // State transition matrix (linearized)
    Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
    // (Simplified - full F would include coupling terms)
    // For covariance propagation, first-order approximation is sufficient

    // Noise Jacobian
    Eigen::Matrix<double, 9, 12> G = Eigen::Matrix<double, 9, 12>::Zero();

    Matrix3d R = delta_R_.toRotationMatrix();

    // ∂ΔR/∂η_g (gyro noise)
    G.block<3, 3>(0, 0) = -R * dt;

    // ∂Δv/∂η_a (accel noise)
    G.block<3, 3>(3, 3) = -R * dt;

    // ∂Δv/∂η_bg (gyro bias noise, affects velocity via rotation error)
    G.block<3, 3>(3, 6) = Matrix3d::Zero();  // Second-order effect, negligible

    // ∂Δv/∂η_ba (accel bias noise)
    G.block<3, 3>(3, 9) = -R * dt;

    // ∂Δp/∂η_g (gyro noise, affects position via rotation error)
    G.block<3, 3>(6, 0) = Matrix3d::Zero();  // Second-order effect

    // ∂Δp/∂η_a (accel noise)
    G.block<3, 3>(6, 3) = -0.5 * R * dt * dt;

    // ∂Δp/∂η_bg
    G.block<3, 3>(6, 6) = Matrix3d::Zero();

    // ∂Δp/∂η_ba
    G.block<3, 3>(6, 9) = -0.5 * R * dt * dt;

    // Covariance propagation
    // P = F * P * F^T + G * Q * G^T
    covariance_ = F * covariance_ * F.transpose() + G * Q * G.transpose();
}

void ImuPreintegration::update_bias(const Vector3d& new_bias_gyro,
                                     const Vector3d& new_bias_accel) {
    // Bias change
    Vector3d d_bg = new_bias_gyro - bias_gyro_;
    Vector3d d_ba = new_bias_accel - bias_accel_;

    // First-order correction (Forster et al. Eq. (44))
    // ΔR' = ΔR * Exp(dR/dbg * d_bg)
    Vector3d theta_correction = dR_dbg_ * d_bg;
    Quaterniond dR_correction = quaternion_exp(theta_correction);
    delta_R_ = delta_R_ * dR_correction;
    delta_R_.normalize();

    // Δv' = Δv + dv/dbg * d_bg + dv/dba * d_ba
    delta_v_ += dv_dbg_ * d_bg + dv_dba_ * d_ba;

    // Δp' = Δp + dp/dbg * d_bg + dp/dba * d_ba
    delta_p_ += dp_dbg_ * d_bg + dp_dba_ * d_ba;

    // Update biases
    bias_gyro_ = new_bias_gyro;
    bias_accel_ = new_bias_accel;
}

PreintegratedImu ImuPreintegration::get_result() const {
    PreintegratedImu result;

    result.start_time_ns = start_time_ns_;
    result.end_time_ns = end_time_ns_;
    result.dt = delta_t_;

    result.delta_R = delta_R_;
    result.delta_v = delta_v_;
    result.delta_p = delta_p_;

    result.dR_dbg = dR_dbg_;
    result.dv_dbg = dv_dbg_;
    result.dv_dba = dv_dba_;
    result.dp_dbg = dp_dbg_;
    result.dp_dba = dp_dba_;

    result.covariance = covariance_;
    result.noise_params = noise_params_;

    return result;
}

Matrix3d ImuPreintegration::skew(const Vector3d& v) {
    Matrix3d m;
    m <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return m;
}

}  // namespace fusion
