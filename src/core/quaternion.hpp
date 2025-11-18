/**
 * @file quaternion.hpp
 * @brief Quaternion utilities for rotation representation in error-state EKF
 *
 * Purpose: Provides quaternion operations for SO(3) rotations. Critical for
 * error-state EKF where orientation is represented as quaternion but error
 * as 3D axis-angle vector.
 *
 * References:
 * - Joan Solà: "Quaternion kinematics for the error-state Kalman filter"
 *   https://arxiv.org/abs/1711.02508
 * - Eigen Geometry: https://eigen.tuxfamily.org/dox/group__Geometry__Module.html
 *
 * Sample Input:
 *   - omega = [0.1, 0, 0] rad/s (axis-angle)
 *   - q = [w=0.707, x=0.707, y=0, z=0] (90° rotation about X)
 *
 * Expected Output:
 *   - quaternion_exp(omega) -> Quaterniond representing small rotation
 *   - quaternion_log(q) -> [π/2, 0, 0] (axis-angle)
 *   - quaternion_to_rotation_matrix(q) -> 3×3 orthogonal matrix
 */

#ifndef FUSION_CORE_QUATERNION_HPP
#define FUSION_CORE_QUATERNION_HPP

#include "types.hpp"
#include <cmath>
#include <utility>

namespace fusion {

/**
 * @brief Exponential map: axis-angle -> quaternion
 *
 * Converts a 3D axis-angle vector (omega) to a quaternion.
 * For small rotations (||omega|| << 1), this is the first-order approximation
 * used in error-state injection.
 *
 * @param omega Axis-angle vector [rad]
 * @return Quaternion representing rotation
 */
Quaterniond quaternion_exp(const Vector3d& omega);

/**
 * @brief Logarithm map: quaternion -> axis-angle
 *
 * Inverse of quaternion_exp. Extracts the axis-angle representation.
 *
 * @param q Unit quaternion
 * @return Axis-angle vector [rad]
 */
Vector3d quaternion_log(const Quaterniond& q);

/**
 * @brief Create quaternion from axis-angle representation
 *
 * @param axis Rotation axis (unit vector)
 * @param angle Rotation angle [rad]
 * @return Quaternion representing rotation
 */
Quaterniond quaternion_from_axis_angle(const Vector3d& axis, double angle);

/**
 * @brief Convert quaternion to rotation matrix
 *
 * Extracts the 3×3 rotation matrix from a quaternion.
 * Result is guaranteed to be orthogonal (R * R^T = I).
 *
 * @param q Unit quaternion
 * @return 3×3 rotation matrix
 */
Matrix3d quaternion_to_rotation_matrix(const Quaterniond& q);

/**
 * @brief Rotate a vector by a quaternion
 *
 * Performs v' = q * v * q^(-1) efficiently.
 *
 * @param q Quaternion representing rotation
 * @param v Vector to rotate
 * @return Rotated vector
 */
Vector3d quaternion_rotate(const Quaterniond& q, const Vector3d& v);

/**
 * @brief Quaternion multiplication (Hamilton product)
 *
 * Note: Eigen already provides operator* for quaternions, but this
 * function is explicit for clarity in the codebase.
 *
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Product q1 * q2
 */
inline Quaterniond quaternion_multiply(const Quaterniond& q1, const Quaterniond& q2) {
    return q1 * q2;
}

/**
 * @brief Normalize quaternion to unit length
 *
 * Essential to prevent drift in integration. Should be called after
 * each quaternion update.
 *
 * @param q Quaternion (potentially non-unit)
 * @return Normalized quaternion
 */
inline Quaterniond quaternion_normalize(const Quaterniond& q) {
    return Quaterniond(q.normalized());
}

/**
 * @brief Compute angular distance between two quaternions
 *
 * Returns the angle (in radians) of the rotation that transforms q1 to q2.
 * This is the geodesic distance on the unit sphere.
 *
 * @param q1 First quaternion
 * @param q2 Second quaternion
 * @return Angular distance [rad], always in [0, π]
 */
double quaternion_distance(const Quaterniond& q1, const Quaterniond& q2);

/**
 * @brief Convert quaternion to axis-angle representation
 *
 * Returns (axis, angle) where axis is unit vector and angle is in [0, π].
 *
 * @param q Unit quaternion
 * @return pair of (axis vector, angle in radians)
 */
std::pair<Vector3d, double> quaternion_to_axis_angle(const Quaterniond& q);

} // namespace fusion

#endif // FUSION_CORE_QUATERNION_HPP
