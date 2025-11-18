/**
 * @file quaternion.cpp
 * @brief Implementation of quaternion utilities
 */

#include "quaternion.hpp"
#include <cmath>

namespace fusion {

Quaterniond quaternion_exp(const Vector3d& omega) {
    // theta = ||omega|| (rotation angle)
    double theta = omega.norm();

    if (theta < 1e-8) {
        // Small angle approximation: q ≈ [1, omega/2]
        // This is the first-order Taylor expansion
        return Quaterniond(1.0,
                          0.5 * omega.x(),
                          0.5 * omega.y(),
                          0.5 * omega.z()).normalized();
    } else {
        // Full exponential map: q = [cos(θ/2), sin(θ/2) * axis]
        double half_theta = 0.5 * theta;
        double s = std::sin(half_theta) / theta;  // Normalized sine

        return Quaterniond(std::cos(half_theta),
                          s * omega.x(),
                          s * omega.y(),
                          s * omega.z());
    }
}

Vector3d quaternion_log(const Quaterniond& q) {
    // Ensure quaternion is normalized
    Quaterniond q_norm = q.normalized();

    // Extract scalar and vector parts
    double w = q_norm.w();
    Vector3d v(q_norm.x(), q_norm.y(), q_norm.z());

    double v_norm = v.norm();

    if (v_norm < 1e-8) {
        // Near identity: omega ≈ 2 * v
        return 2.0 * v;
    } else {
        // Full logarithm: omega = 2 * atan2(||v||, w) * v / ||v||
        double theta = 2.0 * std::atan2(v_norm, w);
        return (theta / v_norm) * v;
    }
}

Quaterniond quaternion_from_axis_angle(const Vector3d& axis, double angle) {
    // Normalize axis
    Vector3d axis_norm = axis.normalized();

    // Construct quaternion: q = [cos(θ/2), sin(θ/2) * axis]
    double half_angle = 0.5 * angle;
    double s = std::sin(half_angle);

    return Quaterniond(std::cos(half_angle),
                      s * axis_norm.x(),
                      s * axis_norm.y(),
                      s * axis_norm.z());
}

Matrix3d quaternion_to_rotation_matrix(const Quaterniond& q) {
    // Use Eigen's built-in conversion (guaranteed orthogonal)
    return q.normalized().toRotationMatrix();
}

Vector3d quaternion_rotate(const Quaterniond& q, const Vector3d& v) {
    // Efficient quaternion-vector rotation: v' = q * v * q^(-1)
    // Eigen provides _transformVector() which is optimized
    return q._transformVector(v);
}

double quaternion_distance(const Quaterniond& q1, const Quaterniond& q2) {
    // Angular distance = angle of rotation from q1 to q2
    // delta_q = q2 * q1^(-1)
    Quaterniond delta_q = q2 * q1.conjugate();

    // Extract angle from delta quaternion
    // angle = 2 * acos(|w|), clamped to [-1, 1]
    double w = delta_q.w();
    w = std::max(-1.0, std::min(1.0, w));  // Clamp for numerical stability

    return 2.0 * std::acos(std::abs(w));
}

std::pair<Vector3d, double> quaternion_to_axis_angle(const Quaterniond& q) {
    // Normalize quaternion
    Quaterniond q_norm = q.normalized();

    // Extract scalar and vector parts
    double w = q_norm.w();
    Vector3d v(q_norm.x(), q_norm.y(), q_norm.z());

    double v_norm = v.norm();

    if (v_norm < 1e-8) {
        // Near identity: zero rotation
        return {Vector3d(1, 0, 0), 0.0};  // Arbitrary axis, zero angle
    }

    // angle = 2 * acos(w)
    // axis = v / ||v||
    double angle = 2.0 * std::acos(std::max(-1.0, std::min(1.0, w)));
    Vector3d axis = v / v_norm;

    return {axis, angle};
}

} // namespace fusion

// ========== Validation Function ==========
#ifdef STANDALONE_VALIDATION

#include <iostream>
#include <iomanip>

using namespace fusion;

int main() {
    std::cout << "========== Quaternion Utilities Validation ==========" << std::endl;

    int total_tests = 0;
    int failed_tests = 0;

    // Test 1: Exp-Log inverse relationship
    total_tests++;
    {
        Vector3d omega(0.1, 0.2, 0.3);  // Small rotation
        Quaterniond q = quaternion_exp(omega);
        Vector3d omega_recovered = quaternion_log(q);

        double error = (omega - omega_recovered).norm();
        if (error > 1e-10) {
            failed_tests++;
            std::cerr << "  ❌ Test 1 FAILED: Exp-Log inverse error = " << error << std::endl;
        } else {
            std::cout << "  ✓ Test 1 PASSED: Exp-Log inverse (error = " << error << ")" << std::endl;
        }
    }

    // Test 2: Rotation matrix orthogonality
    total_tests++;
    {
        Quaterniond q = quaternion_from_axis_angle(Vector3d(1, 0, 0), M_PI / 2);
        Matrix3d R = quaternion_to_rotation_matrix(q);
        Matrix3d I = R * R.transpose();

        double error = (I - Matrix3d::Identity()).norm();
        if (error > 1e-10) {
            failed_tests++;
            std::cerr << "  ❌ Test 2 FAILED: Rotation matrix not orthogonal, error = " << error << std::endl;
        } else {
            std::cout << "  ✓ Test 2 PASSED: Rotation matrix orthogonal (error = " << error << ")" << std::endl;
        }
    }

    // Test 3: 90° rotation about X-axis
    total_tests++;
    {
        Quaterniond q = quaternion_from_axis_angle(Vector3d(1, 0, 0), M_PI / 2);
        Vector3d v(0, 1, 0);  // Y-axis
        Vector3d v_rotated = quaternion_rotate(q, v);
        Vector3d expected(0, 0, 1);  // Should rotate to Z-axis

        double error = (v_rotated - expected).norm();
        if (error > 1e-10) {
            failed_tests++;
            std::cerr << "  ❌ Test 3 FAILED: 90° rotation error = " << error << std::endl;
            std::cerr << "     Expected: [0, 0, 1], Got: ["
                      << v_rotated.x() << ", " << v_rotated.y() << ", " << v_rotated.z() << "]" << std::endl;
        } else {
            std::cout << "  ✓ Test 3 PASSED: 90° rotation (error = " << error << ")" << std::endl;
        }
    }

    // Test 4: Quaternion normalization
    total_tests++;
    {
        Quaterniond q(2.0, 1.0, 1.0, 1.0);  // Non-unit quaternion
        Quaterniond q_norm = quaternion_normalize(q);

        double norm = q_norm.norm();
        double error = std::abs(norm - 1.0);
        if (error > 1e-10) {
            failed_tests++;
            std::cerr << "  ❌ Test 4 FAILED: Normalization error = " << error << std::endl;
        } else {
            std::cout << "  ✓ Test 4 PASSED: Normalization (norm = " << norm << ")" << std::endl;
        }
    }

    // Test 5: Identity quaternion
    total_tests++;
    {
        Vector3d omega = Vector3d::Zero();
        Quaterniond q = quaternion_exp(omega);

        double error = (q.coeffs() - Quaterniond::Identity().coeffs()).norm();
        if (error > 1e-10) {
            failed_tests++;
            std::cerr << "  ❌ Test 5 FAILED: Identity quaternion error = " << error << std::endl;
        } else {
            std::cout << "  ✓ Test 5 PASSED: Identity quaternion (error = " << error << ")" << std::endl;
        }
    }

    // Final report
    std::cout << "========================================" << std::endl;
    if (failed_tests > 0) {
        std::cerr << "❌ VALIDATION FAILED - " << failed_tests << " of " << total_tests << " tests failed" << std::endl;
        return 1;
    } else {
        std::cout << "✅ VALIDATION PASSED - All " << total_tests << " tests successful" << std::endl;
        return 0;
    }
}

#endif // STANDALONE_VALIDATION
