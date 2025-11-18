/**
 * @file synthetic_gnss.cpp
 * @brief Implementation of synthetic GNSS generator
 */

#include "synthetic_gnss.hpp"
#include <cmath>
#include <chrono>

namespace fusion {

SyntheticGnss::SyntheticGnss(const NoiseParams& params, uint32_t seed)
    : params_(params), normal_dist_(0.0, 1.0)
{
    if (seed == 0) {
        seed = static_cast<uint32_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
    }
    rng_.seed(seed);

    initialize_satellite_constellation();
}

void SyntheticGnss::initialize_satellite_constellation() {
    // Simplified constellation: 12 satellites in circular orbit
    // GPS orbit radius: ~26,560 km from Earth center
    double orbit_radius = 26560e3;  // meters

    // Distribute satellites uniformly in 3 orbital planes
    for (int plane = 0; plane < 3; plane++) {
        double plane_angle = plane * 2.0 * M_PI / 3.0;

        for (int sat = 0; sat < 4; sat++) {
            double sat_angle = sat * 2.0 * M_PI / 4.0;

            // Simplified: satellites in equatorial planes (not inclined)
            Vector3d pos;
            pos.x() = orbit_radius * std::cos(plane_angle + sat_angle);
            pos.y() = orbit_radius * std::sin(plane_angle + sat_angle);
            pos.z() = orbit_radius * 0.3 * std::sin(sat_angle);  // Some Z variation

            satellite_positions_ecef_.push_back(pos);
        }
    }
}

double SyntheticGnss::sample_normal(double mean, double std_dev) {
    return mean + std_dev * normal_dist_(rng_);
}

GnssMeasurement SyntheticGnss::generate(const Vector3d& pos_ecef,
                                       const Vector3d& vel_ecef,
                                       timestamp_t timestamp,
                                       int num_sats) {
    GnssMeasurement meas;
    meas.timestamp_ns = timestamp;
    meas.num_sats = std::min(num_sats, static_cast<int>(satellite_positions_ecef_.size()));

    // Receiver clock bias (random constant for this measurement)
    meas.clock_bias = sample_normal(0.0, 100.0);  // ~100m typical
    meas.clock_drift = sample_normal(0.0, 1.0);

    // Generate measurements for each satellite
    for (int i = 0; i < meas.num_sats; i++) {
        GnssSatellite& sat = meas.sats[i];
        sat.prn = i + 1;
        sat.constellation = 0;  // GPS

        // Satellite position (simplified: static constellation)
        sat.sat_pos_ecef = satellite_positions_ecef_[i];
        sat.sat_vel_ecef = Vector3d::Zero();  // Simplified: static

        // True range: distance from receiver to satellite
        Vector3d los = sat.sat_pos_ecef - pos_ecef;
        double true_range = los.norm();

        // Pseudorange = true range + clock bias + noise
        sat.pseudorange = true_range + meas.clock_bias +
                         sample_normal(0.0, params_.pseudorange_noise);

        // Doppler (simplified: zero relative velocity)
        sat.doppler = sample_normal(0.0, params_.doppler_noise);

        // Carrier-to-noise ratio
        sat.cn0 = sample_normal(params_.cn0_mean, params_.cn0_std);

        // Elevation and azimuth (computed from line-of-sight)
        Vector3d los_unit = los.normalized();
        sat.elevation = std::asin(los_unit.z());
        sat.azimuth = std::atan2(los_unit.y(), los_unit.x());
    }

    return meas;
}

} // namespace fusion
