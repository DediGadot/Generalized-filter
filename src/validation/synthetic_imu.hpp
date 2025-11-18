/**
 * @file synthetic_imu.hpp
 * @brief Synthetic IMU data generator for software-first validation
 *
 * Purpose: Generate realistic IMU measurements with proper noise characteristics
 * based on Allan variance parameters. Critical for testing without hardware.
 *
 * References:
 * - "An Introduction to Inertial Navigation" by Oliver Woodman
 * - Allan Variance: IEEE-STD-952-1997
 * - DESIGN.md: Validation Strategy
 *
 * Sample Input:
 *   - Duration: 10 seconds, Rate: 200 Hz
 *   - Motion: Stationary (static bias + noise)
 *
 * Expected Output:
 *   - 2000 IMU samples with realistic noise
 *   - Mean acceleration ≈ [0, 0, 9.81] m/s²
 *   - Gyro noise std dev ≈ noise_density / √dt
 */

#ifndef FUSION_VALIDATION_SYNTHETIC_IMU_HPP
#define FUSION_VALIDATION_SYNTHETIC_IMU_HPP

#include "core/sensor_types.hpp"
#include "core/quaternion.hpp"
#include <vector>
#include <random>

namespace fusion {

/**
 * @brief Synthetic IMU generator with realistic noise models
 *
 * Implements Allan variance-based noise for MEMS IMU sensors.
 * Typical values are from consumer-grade MEMS (e.g., BMI088, ICM-42688).
 */
class SyntheticImu {
public:
    /**
     * @brief IMU noise parameters (from Allan variance analysis)
     */
    struct NoiseParams {
        // Gyroscope
        double gyro_noise_density;      ///< [rad/s/√Hz] White noise
        double gyro_bias_stability;     ///< [rad/s] Constant bias
        double gyro_random_walk;        ///< [rad/s²/√Hz] Bias random walk

        // Accelerometer
        double accel_noise_density;     ///< [m/s²/√Hz] White noise
        double accel_bias_stability;    ///< [m/s²] Constant bias
        double accel_random_walk;       ///< [m/s³/√Hz] Bias random walk

        // Default: Consumer MEMS IMU (BMI088-like)
        static NoiseParams default_consumer() {
            return NoiseParams{
                .gyro_noise_density = 0.0001,     // 0.1 mdps/√Hz
                .gyro_bias_stability = 0.001,     // ~0.06 deg/s
                .gyro_random_walk = 1e-6,

                .accel_noise_density = 0.001,     // 0.1 mg/√Hz
                .accel_bias_stability = 0.01,     // ~1 mg
                .accel_random_walk = 1e-5
            };
        }
    };

    /**
     * @brief Constructor
     * @param params Noise parameters
     * @param seed Random seed (0 = random)
     */
    explicit SyntheticImu(const NoiseParams& params = NoiseParams::default_consumer(),
                         uint32_t seed = 0);

    /**
     * @brief Generate static IMU samples (stationary sensor)
     *
     * @param num_samples Number of samples to generate
     * @param dt Sample period [s]
     * @return Vector of IMU samples
     */
    std::vector<ImuSample> generate_static(int num_samples, double dt);

    /**
     * @brief Generate IMU samples for pure rotation
     *
     * @param omega Angular velocity [rad/s], body frame
     * @param duration Duration [s]
     * @param dt Sample period [s]
     * @return Vector of IMU samples
     */
    std::vector<ImuSample> generate_rotation(const Vector3d& omega,
                                             double duration,
                                             double dt);

    /**
     * @brief Generate IMU samples for linear motion
     *
     * @param acceleration Linear acceleration [m/s²], navigation frame
     * @param duration Duration [s]
     * @param dt Sample period [s]
     * @return Vector of IMU samples
     */
    std::vector<ImuSample> generate_linear(const Vector3d& acceleration,
                                           double duration,
                                           double dt);

    /**
     * @brief Reset biases (for new trajectory)
     */
    void reset();

private:
    NoiseParams params_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;

    // Current biases (random walk)
    Vector3d gyro_bias_;
    Vector3d accel_bias_;

    // Helper: Add noise to measurement
    Vector3f add_gyro_noise(const Vector3d& true_gyro, double dt);
    Vector3f add_accel_noise(const Vector3d& true_accel, double dt);

    // Helper: Sample from normal distribution
    double sample_normal(double mean, double std_dev);
};

} // namespace fusion

#endif // FUSION_VALIDATION_SYNTHETIC_IMU_HPP
