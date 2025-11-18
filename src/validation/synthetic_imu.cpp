/**
 * @file synthetic_imu.cpp
 * @brief Implementation of synthetic IMU generator
 */

#include "synthetic_imu.hpp"
#include "math/vector_math.hpp"
#include <cmath>
#include <chrono>

namespace fusion {

SyntheticImu::SyntheticImu(const NoiseParams& params, uint32_t seed)
    : params_(params),
      normal_dist_(0.0, 1.0),
      gyro_bias_(Vector3d::Zero()),
      accel_bias_(Vector3d::Zero())
{
    if (seed == 0) {
        seed = static_cast<uint32_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
    }
    rng_.seed(seed);

    // Initialize biases (random within stability bounds)
    for (int i = 0; i < 3; i++) {
        gyro_bias_(i) = sample_normal(0.0, params_.gyro_bias_stability);
        accel_bias_(i) = sample_normal(0.0, params_.accel_bias_stability);
    }
}

void SyntheticImu::reset() {
    gyro_bias_.setZero();
    accel_bias_.setZero();
}

double SyntheticImu::sample_normal(double mean, double std_dev) {
    return mean + std_dev * normal_dist_(rng_);
}

Vector3f SyntheticImu::add_gyro_noise(const Vector3d& true_gyro, double dt) {
    // White noise: σ = noise_density / √dt
    double white_noise_std = params_.gyro_noise_density / std::sqrt(dt);

    // Random walk: bias changes by √dt * random_walk
    double bias_change_std = std::sqrt(dt) * params_.gyro_random_walk;

    Vector3d noisy_gyro = true_gyro;
    for (int i = 0; i < 3; i++) {
        // Update bias (random walk)
        gyro_bias_(i) += sample_normal(0.0, bias_change_std);

        // Add white noise + bias
        noisy_gyro(i) += gyro_bias_(i) + sample_normal(0.0, white_noise_std);
    }

    return noisy_gyro.cast<float>();
}

Vector3f SyntheticImu::add_accel_noise(const Vector3d& true_accel, double dt) {
    double white_noise_std = params_.accel_noise_density / std::sqrt(dt);
    double bias_change_std = std::sqrt(dt) * params_.accel_random_walk;

    Vector3d noisy_accel = true_accel;
    for (int i = 0; i < 3; i++) {
        accel_bias_(i) += sample_normal(0.0, bias_change_std);
        noisy_accel(i) += accel_bias_(i) + sample_normal(0.0, white_noise_std);
    }

    return noisy_accel.cast<float>();
}

std::vector<ImuSample> SyntheticImu::generate_static(int num_samples, double dt) {
    std::vector<ImuSample> samples;
    samples.reserve(num_samples);

    // Stationary: zero angular velocity, gravity in Z
    Vector3d true_gyro(0, 0, 0);
    Vector3d true_accel(0, 0, GRAVITY);  // NED frame: gravity points down

    for (int i = 0; i < num_samples; i++) {
        ImuSample sample;
        sample.timestamp_ns = static_cast<timestamp_t>(i * dt * 1e9);
        sample.gyro = add_gyro_noise(true_gyro, dt);
        sample.accel = add_accel_noise(true_accel, dt);
        sample.flags = 0;

        samples.push_back(sample);
    }

    return samples;
}

std::vector<ImuSample> SyntheticImu::generate_rotation(const Vector3d& omega,
                                                       double duration,
                                                       double dt) {
    int num_samples = static_cast<int>(duration / dt);
    std::vector<ImuSample> samples;
    samples.reserve(num_samples);

    // Constant rotation: angular velocity = omega, no linear acceleration
    // (except gravity in body frame, which rotates)

    Quaterniond orientation = Quaterniond::Identity();
    Vector3d gravity_nav(0, 0, GRAVITY);  // NED frame

    for (int i = 0; i < num_samples; i++) {
        // Gravity in body frame (rotates as sensor rotates)
        Vector3d gravity_body = orientation.inverse() * gravity_nav;

        ImuSample sample;
        sample.timestamp_ns = static_cast<timestamp_t>(i * dt * 1e9);
        sample.gyro = add_gyro_noise(omega, dt);
        sample.accel = add_accel_noise(gravity_body, dt);
        sample.flags = 0;

        samples.push_back(sample);

        // Update orientation for next sample
        Quaterniond dq = quaternion_exp(omega * dt);
        orientation = orientation * dq;
        orientation.normalize();
    }

    return samples;
}

std::vector<ImuSample> SyntheticImu::generate_linear(const Vector3d& acceleration,
                                                     double duration,
                                                     double dt) {
    int num_samples = static_cast<int>(duration / dt);
    std::vector<ImuSample> samples;
    samples.reserve(num_samples);

    // Linear motion: constant acceleration in navigation frame
    Vector3d true_gyro(0, 0, 0);
    Vector3d gravity_nav(0, 0, GRAVITY);

    for (int i = 0; i < num_samples; i++) {
        // Specific force = acceleration - gravity (what IMU measures)
        Vector3d specific_force = acceleration - gravity_nav;

        ImuSample sample;
        sample.timestamp_ns = static_cast<timestamp_t>(i * dt * 1e9);
        sample.gyro = add_gyro_noise(true_gyro, dt);
        sample.accel = add_accel_noise(specific_force, dt);
        sample.flags = 0;

        samples.push_back(sample);
    }

    return samples;
}

} // namespace fusion

// ========== Validation Function ==========
#ifdef STANDALONE_VALIDATION

#include <iostream>
#include <iomanip>
#include "math/vector_math.hpp"

using namespace fusion;

int main() {
    std::cout << "========== Synthetic IMU Validation ==========" << std::endl;

    int total_tests = 0;
    int failed_tests = 0;

    SyntheticImu imu_gen;

    // Test 1: Static samples - mean acceleration should be ~gravity
    total_tests++;
    {
        auto samples = imu_gen.generate_static(1000, 0.005);  // 5 seconds at 200 Hz

        std::vector<Vector3d> accels;
        for (const auto& s : samples) {
            accels.push_back(s.accel.cast<double>());
        }

        Vector3d mean_accel = compute_mean(accels);
        Vector3d expected(0, 0, GRAVITY);
        double error = (mean_accel - expected).norm();

        if (error > 0.1) {  // Allow 0.1 m/s² error
            failed_tests++;
            std::cerr << "  ❌ Test 1 FAILED: Static mean accel error = " << error << " m/s²" << std::endl;
            std::cerr << "     Expected: [0, 0, " << GRAVITY << "]" << std::endl;
            std::cerr << "     Got: [" << mean_accel.x() << ", "
                      << mean_accel.y() << ", " << mean_accel.z() << "]" << std::endl;
        } else {
            std::cout << "  ✓ Test 1 PASSED: Static mean = [" << mean_accel.x() << ", "
                      << mean_accel.y() << ", " << mean_accel.z() << "] (error = "
                      << error << " m/s²)" << std::endl;
        }
    }

    // Test 2: Rotation - integrate gyro should give expected angle
    total_tests++;
    {
        imu_gen.reset();
        double omega_z = 0.5;  // 0.5 rad/s about Z
        double duration = 2.0 * M_PI / omega_z;  // One full rotation
        auto samples = imu_gen.generate_rotation(Vector3d(0, 0, omega_z), duration, 0.005);

        // Integrate gyro to get total rotation
        double total_rotation = 0.0;
        for (const auto& s : samples) {
            total_rotation += s.gyro.z() * 0.005;
        }

        double expected_rotation = 2.0 * M_PI;
        double error = std::abs(total_rotation - expected_rotation);

        if (error > 0.1) {  // Allow 0.1 rad error
            failed_tests++;
            std::cerr << "  ❌ Test 2 FAILED: Rotation integration error = " << error << " rad" << std::endl;
            std::cerr << "     Expected: " << expected_rotation << " rad (2π)" << std::endl;
            std::cerr << "     Got: " << total_rotation << " rad" << std::endl;
        } else {
            std::cout << "  ✓ Test 2 PASSED: Rotation = " << total_rotation << " rad (error = "
                      << error << " rad)" << std::endl;
        }
    }

    // Test 3: Sample count and timing
    total_tests++;
    {
        int expected_samples = 2000;
        auto samples = imu_gen.generate_static(expected_samples, 0.005);

        if (samples.size() != static_cast<size_t>(expected_samples)) {
            failed_tests++;
            std::cerr << "  ❌ Test 3 FAILED: Sample count mismatch" << std::endl;
            std::cerr << "     Expected: " << expected_samples << std::endl;
            std::cerr << "     Got: " << samples.size() << std::endl;
        } else {
            std::cout << "  ✓ Test 3 PASSED: Generated " << samples.size() << " samples" << std::endl;
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
