/**
 * @file vector_math.hpp
 * @brief Common vector and matrix operations for sensor fusion
 *
 * Purpose: Provides utility functions for vector operations used throughout
 * the filter. Thin wrappers around Eigen for code clarity.
 *
 * References:
 * - Eigen Dense: https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html
 *
 * Sample Input:
 *   - v = [1, 2, 3]
 * Expected Output:
 *   - norm(v) = 3.742
 *   - normalize(v) = [0.267, 0.535, 0.802]
 *   - skew(v) = [[0, -3, 2], [3, 0, -1], [-2, 1, 0]]
 */

#ifndef FUSION_MATH_VECTOR_MATH_HPP
#define FUSION_MATH_VECTOR_MATH_HPP

#include "core/types.hpp"

namespace fusion {

/**
 * @brief Compute norm (magnitude) of a vector
 */
inline double vector_norm(const Vector3d& v) {
    return v.norm();
}

/**
 * @brief Normalize a vector to unit length
 */
inline Vector3d vector_normalize(const Vector3d& v) {
    return v.normalized();
}

/**
 * @brief Create skew-symmetric matrix from vector
 *
 * For cross product: skew(a) * b = a × b
 * Used in Jacobian computations.
 *
 * @param v Input vector
 * @return 3×3 skew-symmetric matrix [v]×
 */
Matrix3d skew_symmetric(const Vector3d& v);

/**
 * @brief Compute cross product using skew-symmetric matrix
 *
 * Alternative to Eigen's cross() function, useful for Jacobians.
 */
inline Vector3d cross_product(const Vector3d& a, const Vector3d& b) {
    return a.cross(b);
}

/**
 * @brief Compute dot product
 */
inline double dot_product(const Vector3d& a, const Vector3d& b) {
    return a.dot(b);
}

/**
 * @brief Compute mean of vector samples
 */
Vector3d compute_mean(const std::vector<Vector3d>& samples);

/**
 * @brief Compute variance of vector samples
 */
Vector3d compute_variance(const std::vector<Vector3d>& samples);

/**
 * @brief Right Jacobian of SO(3)
 *
 * Used in rotation error propagation and Jacobian computation.
 * For small angles θ: Jr(θ) ≈ I - 0.5*[θ]× + ...
 *
 * Reference: "A micro Lie theory for state estimation in robotics"
 *            Sola et al., 2018
 *
 * @param theta Rotation vector (axis-angle representation)
 * @return 3×3 right Jacobian matrix
 */
Matrix3d right_jacobian(const Vector3d& theta);

/**
 * @brief Left Jacobian of SO(3)
 *
 * Jl(θ) = Jr(-θ)
 *
 * @param theta Rotation vector
 * @return 3×3 left Jacobian matrix
 */
inline Matrix3d left_jacobian(const Vector3d& theta) {
    return right_jacobian(-theta);
}

} // namespace fusion

#endif // FUSION_MATH_VECTOR_MATH_HPP
