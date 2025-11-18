// Magnetometer Measurement Update Implementation
#include "filter/mag_update.hpp"
#include "math/vector_math.hpp"

#include <cmath>

namespace fusion {

// =============================================================================
// World Magnetic Model (Simplified)
// =============================================================================

void WorldMagneticModel::set_location(double latitude_deg, double longitude_deg, double altitude_m) {
    latitude_deg_ = latitude_deg;
    longitude_deg_ = longitude_deg;
    altitude_m_ = altitude_m;

    // Compute magnetic field parameters
    compute_magnetic_field();
}

void WorldMagneticModel::compute_magnetic_field() {
    // Simplified World Magnetic Model
    //
    // Full WMM2025 uses spherical harmonics with 720+ coefficients.
    // This is a simplified model for testing/prototyping.
    //
    // For production, integrate full NOAA WMM2025:
    // https://www.ncei.noaa.gov/products/world-magnetic-model
    //
    // Typical values:
    // - Declination: -20° to +20° (varies by location)
    // - Inclination: -90° (south pole) to +90° (north pole)
    // - Field strength: 25-65 μT (varies by location)

    // Convert latitude to radians
    double lat_rad = latitude_deg_ * M_PI / 180.0;

    // === Declination (simplified: varies with longitude) ===
    // Real WMM has complex spatial variation
    // Simplified: use small variation based on longitude
    declination_rad_ = 0.05 * std::sin(longitude_deg_ * M_PI / 180.0);  // ±3° variation

    // === Inclination (simplified: based on latitude) ===
    // Inclination ≈ arctan(2 * tan(latitude)) for dipole model
    // Ranges from -90° (south magnetic pole) to +90° (north magnetic pole)
    inclination_rad_ = std::atan(2.0 * std::tan(lat_rad));

    // === Field strength (simplified: dipole model) ===
    // Total field strength varies with latitude
    // Typical: 25 μT (equator) to 65 μT (poles)
    // Dipole approximation: B = B0 * sqrt(1 + 3*sin²(lat))
    double B0 = 30.0;  // Field strength at equator [μT]
    double sin_lat = std::sin(lat_rad);
    field_strength_ = B0 * std::sqrt(1.0 + 3.0 * sin_lat * sin_lat);

    // Altitude correction (field decreases with altitude)
    // Roughly -0.03 μT per km
    double altitude_km = altitude_m_ / 1000.0;
    field_strength_ -= 0.03 * altitude_km;
}

Vector3d WorldMagneticModel::get_reference_field_ned() const {
    // Construct magnetic field vector in NED frame
    //
    // The magnetic field has:
    // - Horizontal component: points towards magnetic north (rotated by declination)
    // - Vertical component: points down (positive in NED down direction)
    //
    // Total field = [H*cos(D), H*sin(D), V]
    // where:
    //   H = horizontal component = B * cos(I)
    //   V = vertical component = B * sin(I)
    //   D = declination
    //   I = inclination
    //   B = total field strength

    double horizontal = field_strength_ * std::cos(inclination_rad_);
    double vertical = field_strength_ * std::sin(inclination_rad_);

    // NED components
    double north = horizontal * std::cos(declination_rad_);
    double east = horizontal * std::sin(declination_rad_);
    double down = vertical;

    return Vector3d(north, east, down);
}

// =============================================================================
// Magnetometer Update
// =============================================================================

MagUpdate::MagUpdate(double nominal_noise_std, double adaptive_noise_factor)
    : nominal_noise_std_(nominal_noise_std),
      adaptive_noise_factor_(adaptive_noise_factor),
      last_innovation_(Vector3d::Zero()),
      last_adaptive_factor_(1.0) {
}

void MagUpdate::set_location(double latitude_deg, double longitude_deg, double altitude_m) {
    wmm_.set_location(latitude_deg, longitude_deg, altitude_m);
}

bool MagUpdate::update(
    EkfState& state,
    const Vector3d& mag_body_measured,
    bool enable_gating) {

    // === 1. Get reference field from WMM ===
    Vector3d mag_nav_reference = wmm_.get_reference_field_ned();

    // === 2. Predict measurement: h(x) = R_nb^T * m_n ===
    // Rotate reference field from navigation to body frame
    Matrix3d R_nb = state.attitude().toRotationMatrix();
    Vector3d mag_body_predicted = R_nb.transpose() * mag_nav_reference;

    // === 3. Innovation: y = z - h(x) ===
    Vector3d innovation = mag_body_measured - mag_body_predicted;
    last_innovation_ = innovation;

    // === 4. Compute Jacobian H ===
    Eigen::Matrix<double, 3, 15> H = compute_jacobian(mag_body_predicted);

    // === 5. Adaptive Measurement Noise ===
    Matrix3d R = compute_adaptive_noise(mag_body_measured);

    // === 6. Perform Generic Update ===
    // Chi-square threshold for 3 DOF, 95% confidence: 7.815
    bool accepted = update_.update(state, innovation, H, R, enable_gating, 7.815);

    return accepted;
}

bool MagUpdate::is_magnetic_disturbance(const Vector3d& mag_body_measured) const {
    // Check if measured field strength deviates significantly from reference
    double measured_norm = mag_body_measured.norm();
    double reference_norm = wmm_.get_field_strength();

    // Compute relative deviation
    double deviation = std::abs(measured_norm - reference_norm) / reference_norm;

    // Threshold: 20% deviation indicates likely disturbance
    // (e.g., near metal, electronics, magnets)
    return (deviation > 0.2);
}

Matrix3d MagUpdate::compute_adaptive_noise(const Vector3d& mag_body_measured) const {
    // Base noise covariance (isotropic)
    double base_variance = nominal_noise_std_ * nominal_noise_std_;

    // Compute deviation from reference field strength
    double measured_norm = mag_body_measured.norm();
    double reference_norm = wmm_.get_field_strength();
    double deviation = std::abs(measured_norm - reference_norm) / reference_norm;

    // Adaptive scaling factor: increase noise if deviation detected
    // k = 1 + α * deviation
    // where α is the adaptive_noise_factor (default: 5.0)
    double k = 1.0 + adaptive_noise_factor_ * deviation;

    // Store for diagnostics
    const_cast<MagUpdate*>(this)->last_adaptive_factor_ = k;

    // Adaptive variance: σ²_adaptive = k² * σ²_base
    double adaptive_variance = k * k * base_variance;

    // Return isotropic covariance matrix
    return Matrix3d::Identity() * adaptive_variance;
}

Eigen::Matrix<double, 3, 15> MagUpdate::compute_jacobian(const Vector3d& predicted_mag_body) const {
    // Measurement Jacobian H = ∂h/∂δx
    //
    // For magnetometer measurement:
    //   h(x) = R_nb^T * m_n
    //
    // Linearized about current state:
    //   δh = ∂h/∂δθ * δθ + ∂h/∂δv * δv + ... (only δθ matters)
    //
    // Derivation (error-state formulation):
    //   R_nb(δθ) ≈ R_nb * (I + [δθ]×)    // Right-side perturbation
    //   h(δθ) = (I + [δθ]×)^T * R_nb^T * m_n
    //         = (I - [δθ]×) * R_nb^T * m_n    // Since [δθ]× is skew-symmetric
    //         = R_nb^T * m_n - [δθ]× * R_nb^T * m_n
    //         = h_nominal - [δθ]× * h_nominal
    //         = h_nominal - [h_nominal]× * δθ    // Rewrite using cross product identity
    //
    // Therefore:
    //   ∂h/∂δθ = -[h_nominal]× = -[predicted_mag_body]×
    //
    // All other states (velocity, position, biases) don't affect magnetometer measurement:
    //   ∂h/∂δv = 0, ∂h/∂δp = 0, ∂h/∂δb_g = 0, ∂h/∂δb_a = 0

    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double, 3, 15>::Zero();

    // ∂h/∂δθ = -[predicted_mag_body]× (see derivation above)
    H.block<3, 3>(0, ErrorState::DTHETA) = -skew_symmetric(predicted_mag_body);

    return H;
}

}  // namespace fusion
