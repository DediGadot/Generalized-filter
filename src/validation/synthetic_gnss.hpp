/**
 * @file synthetic_gnss.hpp
 * @brief Synthetic GNSS data generator for software-first validation
 *
 * Purpose: Generate realistic GNSS pseudorange measurements from known positions.
 * Simplified model for Phase 0 validation (full GPS simulator in future phases).
 *
 * References:
 * - "Principles of GNSS" by Paul Groves
 * - DESIGN.md: GNSS tightly-coupled integration
 *
 * Sample Input:
 *   - Position ECEF: [4e6, 0, 4e6] m (approximate mid-latitude)
 *   - 8 satellites visible
 *
 * Expected Output:
 *   - GnssMeasurement with pseudoranges ~20M meters
 *   - Realistic noise and multipath errors
 */

#ifndef FUSION_VALIDATION_SYNTHETIC_GNSS_HPP
#define FUSION_VALIDATION_SYNTHETIC_GNSS_HPP

#include "core/sensor_types.hpp"
#include <random>

namespace fusion {

/**
 * @brief Simplified GNSS simulator for Phase 0 validation
 *
 * Note: This is a simplified model. Full GPS signal simulation
 * (ephemeris, ionosphere, troposphere) is beyond Phase 0 scope.
 */
class SyntheticGnss {
public:
    /**
 * @brief GNSS noise parameters
     */
    struct NoiseParams {
        double pseudorange_noise;   ///< [m] Standard deviation
        double doppler_noise;       ///< [m/s] Standard deviation
        double cn0_mean;            ///< [dB-Hz] Mean carrier-to-noise
        double cn0_std;             ///< [dB-Hz] Standard deviation

        static NoiseParams default_params() {
            return NoiseParams{
                .pseudorange_noise = 5.0,      // 5 meters (typical urban)
                .doppler_noise = 0.1,          // 0.1 m/s
                .cn0_mean = 42.0,              // 42 dB-Hz (good signal)
                .cn0_std = 3.0
            };
        }
    };

    explicit SyntheticGnss(const NoiseParams& params = NoiseParams::default_params(),
                          uint32_t seed = 0);

    /**
     * @brief Generate GNSS measurement from known position
     *
     * @param pos_ecef Position in ECEF [m]
     * @param vel_ecef Velocity in ECEF [m/s]
     * @param timestamp Timestamp [ns]
     * @param num_sats Number of satellites (4-12 typical)
     * @return GNSS measurement
     */
    GnssMeasurement generate(const Vector3d& pos_ecef,
                             const Vector3d& vel_ecef,
                             timestamp_t timestamp,
                             int num_sats = 8);

private:
    NoiseParams params_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;

    // Simplified satellite constellation (fixed positions for Phase 0)
    std::vector<Vector3d> satellite_positions_ecef_;

    void initialize_satellite_constellation();
    double sample_normal(double mean, double std_dev);
};

} // namespace fusion

#endif // FUSION_VALIDATION_SYNTHETIC_GNSS_HPP
