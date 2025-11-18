/**
 * @file vector_math.cpp
 * @brief Implementation of vector math utilities
 */

#include "vector_math.hpp"
#include <cmath>

namespace fusion {

Matrix3d skew_symmetric(const Vector3d& v) {
    Matrix3d skew;
    skew <<     0, -v.z(),  v.y(),
            v.z(),      0, -v.x(),
           -v.y(),  v.x(),      0;
    return skew;
}

Vector3d compute_mean(const std::vector<Vector3d>& samples) {
    if (samples.empty()) {
        return Vector3d::Zero();
    }

    Vector3d sum = Vector3d::Zero();
    for (const auto& sample : samples) {
        sum += sample;
    }
    return sum / static_cast<double>(samples.size());
}

Vector3d compute_variance(const std::vector<Vector3d>& samples) {
    if (samples.empty()) {
        return Vector3d::Zero();
    }

    Vector3d mean = compute_mean(samples);
    Vector3d var = Vector3d::Zero();

    for (const auto& sample : samples) {
        Vector3d diff = sample - mean;
        var += diff.cwiseProduct(diff);  // Element-wise square
    }

    return var / static_cast<double>(samples.size());
}

Matrix3d right_jacobian(const Vector3d& theta) {
    // Right Jacobian of SO(3)
    // Jr(θ) = I - (1-cos|θ|)/|θ|² [θ]× + (|θ|-sin|θ|)/|θ|³ [θ]×²
    //
    // For small θ: Jr(θ) ≈ I - 0.5*[θ]× + (1/6)*[θ]×²

    const double theta_norm = theta.norm();

    // Small angle approximation
    if (theta_norm < 1e-8) {
        // Jr ≈ I - 0.5*[θ]×
        return Matrix3d::Identity() - 0.5 * skew_symmetric(theta);
    }

    // Full formula
    const double theta_norm_sq = theta_norm * theta_norm;
    const double theta_norm_cu = theta_norm_sq * theta_norm;

    const Matrix3d theta_skew = skew_symmetric(theta);
    const Matrix3d theta_skew_sq = theta_skew * theta_skew;

    const double a = (1.0 - std::cos(theta_norm)) / theta_norm_sq;
    const double b = (theta_norm - std::sin(theta_norm)) / theta_norm_cu;

    return Matrix3d::Identity() - a * theta_skew + b * theta_skew_sq;
}

} // namespace fusion
