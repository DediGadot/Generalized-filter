/**
 * @file types.hpp
 * @brief Core type definitions using Eigen library for sensor fusion filter
 *
 * Purpose: Provides strongly-typed aliases for all mathematical types used
 * throughout the filter. Uses Eigen for ARM NEON optimization and type safety.
 *
 * References:
 * - Eigen: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
 * - DESIGN.md: Decision 1 (C++ with Eigen), Decision 2 (Mixed precision)
 *
 * Sample Input: N/A (type definitions only)
 * Expected Output: Compile-time type safety for all mathematical operations
 */

#ifndef FUSION_CORE_TYPES_HPP
#define FUSION_CORE_TYPES_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdint>

namespace fusion {

// ========== Double Precision (for state and covariance) ==========
// Rationale: Numerical stability for error-state EKF (DESIGN.md Decision 2)

using Vector3d = Eigen::Vector3d;
using Quaterniond = Eigen::Quaterniond;
using Matrix3d = Eigen::Matrix3d;

// EKF state dimension: [δθ(3) δv(3) δp(3) δbg(3) δba(3)] = 15
constexpr int STATE_DIM = 15;

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

// Fixed-size state matrices
using Vector15d = Eigen::Matrix<double, 15, 1>;
using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Matrix15x12d = Eigen::Matrix<double, 15, 12>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;

// ========== Single Precision (for sensor measurements) ==========
// Rationale: Sensor data is inherently noisy, Float32 sufficient (DESIGN.md Decision 2)

using Vector3f = Eigen::Vector3f;
using Matrix3f = Eigen::Matrix3f;

// ========== Constants ==========

// Gravity (NED frame, pointing down)
constexpr double GRAVITY = 9.80665;  // m/s² (standard gravity)

// Maximum number of GNSS satellites
constexpr int MAX_GNSS_SATS = 32;

// Timestamp type (nanoseconds since epoch)
using timestamp_t = int64_t;

} // namespace fusion

#endif // FUSION_CORE_TYPES_HPP
