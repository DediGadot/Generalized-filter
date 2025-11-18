/**
 * @file sensor_types.hpp
 * @brief Sensor data structures with mixed precision (DESIGN.md Decision 2)
 *
 * Purpose: Define all sensor measurement structures. Uses Float32 for sensor
 * data (inherently noisy) and Float64 for positions in ECEF (6M meter scale).
 *
 * References:
 * - DESIGN.md: Data Structures section
 * - Android Sensor HAL: https://source.android.com/devices/sensors
 *
 * Sample Input:
 *   - IMU at 200 Hz with gyro = [0.01, 0.02, 0.03] rad/s
 *   - GNSS with 8 satellites, pseudoranges ~20M meters
 *
 * Expected Output:
 *   - Packed structs suitable for lock-free queue transmission
 *   - Cache-aligned for performance
 */

#ifndef FUSION_CORE_SENSOR_TYPES_HPP
#define FUSION_CORE_SENSOR_TYPES_HPP

#include "types.hpp"
#include <cstdint>
#include <vector>

namespace fusion {

// ========== IMU Sample ==========

/**
 * @brief Single IMU measurement (accelerometer + gyroscope)
 *
 * Size: 32 bytes (cache-friendly)
 * Frequency: 200 Hz (5ms period)
 */
struct alignas(16) ImuSample {
    timestamp_t timestamp_ns;   ///< Nanosecond timestamp (CLOCK_MONOTONIC)
    Vector3f gyro;              ///< Angular velocity [rad/s], body frame
    Vector3f accel;             ///< Linear acceleration [m/s²], body frame
    uint8_t flags;              ///< Status flags (saturation, error, etc.)
    uint8_t padding[7];         ///< Padding for alignment

    ImuSample()
        : timestamp_ns(0), gyro(Vector3f::Zero()), accel(Vector3f::Zero()),
          flags(0), padding{0} {}
};

static_assert(sizeof(ImuSample) == 48, "ImuSample size mismatch");

// ========== GNSS Measurement ==========

/**
 * @brief Single GNSS satellite measurement
 */
struct GnssSatellite {
    uint8_t prn;                ///< Satellite PRN (ID)
    uint8_t constellation;      ///< GPS=0, GLONASS=1, Galileo=2, BeiDou=3
    uint8_t padding[6];         ///< Alignment

    double pseudorange;         ///< Pseudorange [m] (FLOAT64 for precision)
    float doppler;              ///< Doppler shift [m/s]
    float cn0;                  ///< Carrier-to-noise ratio [dB-Hz]
    float elevation;            ///< Elevation angle [rad]
    float azimuth;              ///< Azimuth angle [rad]

    Vector3d sat_pos_ecef;      ///< Satellite position ECEF [m] (FLOAT64)
    Vector3d sat_vel_ecef;      ///< Satellite velocity ECEF [m/s] (FLOAT64)

    GnssSatellite()
        : prn(0), constellation(0), padding{0},
          pseudorange(0.0), doppler(0.0f), cn0(0.0f),
          elevation(0.0f), azimuth(0.0f),
          sat_pos_ecef(Vector3d::Zero()), sat_vel_ecef(Vector3d::Zero()) {}
};

/**
 * @brief GNSS measurement (tightly-coupled, multiple satellites)
 *
 * Size: ~3 KB worst case (32 satellites)
 * Frequency: 1-10 Hz
 */
struct GnssMeasurement {
    timestamp_t timestamp_ns;
    uint8_t num_sats;           ///< Number of satellites (1-32)
    uint8_t padding[7];

    GnssSatellite sats[MAX_GNSS_SATS];

    double clock_bias;          ///< Receiver clock bias [m]
    double clock_drift;         ///< Receiver clock drift [m/s]

    GnssMeasurement()
        : timestamp_ns(0), num_sats(0), padding{0},
          clock_bias(0.0), clock_drift(0.0) {}
};

// ========== Magnetometer Measurement ==========

/**
 * @brief Magnetometer measurement
 *
 * Size: 32 bytes
 * Frequency: 50 Hz
 */
struct alignas(16) MagMeasurement {
    timestamp_t timestamp_ns;
    Vector3f mag;               ///< Magnetic field [Gauss], body frame
    float temperature;          ///< Sensor temperature [°C]
    uint8_t flags;              ///< Status flags
    uint8_t padding[7];

    MagMeasurement()
        : timestamp_ns(0), mag(Vector3f::Zero()),
          temperature(0.0f), flags(0), padding{0} {}
};

static_assert(sizeof(MagMeasurement) == 32, "MagMeasurement size mismatch");

// ========== Location Measurement (Android Fused Location) ==========

/**
 * @brief Android Fused Location measurement
 *
 * Pre-computed position from FusedLocationProvider API.
 * Combines GPS, WiFi, cell towers, and BLE for robust positioning.
 *
 * Size: 80 bytes (cache-friendly)
 * Frequency: 0.5-5 Hz (configurable via Priority)
 */
struct alignas(16) LocationMeasurement {
    timestamp_t timestamp_ns;       ///< Measurement timestamp (CLOCK_MONOTONIC)

    // Position in WGS84 geodetic coordinates
    double latitude_deg;            ///< Latitude [-90, 90] degrees
    double longitude_deg;           ///< Longitude [-180, 180] degrees
    double altitude_m;              ///< Altitude above WGS84 ellipsoid [m]

    // Position in local NED frame (computed from WGS84)
    Vector3d position_ned;          ///< Position [North, East, Down] in meters

    // Uncertainty estimates (from Android FusedLocationProvider)
    float horizontal_accuracy_m;    ///< Horizontal accuracy (68% confidence) [m]
    float vertical_accuracy_m;      ///< Vertical accuracy (68% confidence) [m]
    float speed_mps;                ///< Speed [m/s] (optional)
    float speed_accuracy_mps;       ///< Speed accuracy [m/s] (optional)

    // Metadata
    uint8_t provider_flags;         ///< Bit flags: GPS=1, WiFi=2, Cell=4, BLE=8
    uint8_t num_satellites;         ///< Number of GPS satellites used (if available)
    uint8_t padding[6];             ///< Alignment padding

    LocationMeasurement()
        : timestamp_ns(0),
          latitude_deg(0.0), longitude_deg(0.0), altitude_m(0.0),
          position_ned(Vector3d::Zero()),
          horizontal_accuracy_m(999.0f),
          vertical_accuracy_m(999.0f),
          speed_mps(0.0f),
          speed_accuracy_mps(999.0f),
          provider_flags(0),
          num_satellites(0),
          padding{0} {}
};

static_assert(sizeof(LocationMeasurement) == 80, "LocationMeasurement size mismatch");

// ========== IMU Noise Parameters ==========

/**
 * @brief IMU noise parameters for preintegration
 *
 * Derived from Allan variance analysis of the IMU.
 * Units follow convention from Forster et al.
 */
struct ImuNoiseParams {
    double gyro_noise_density;      ///< [rad/s/√Hz] Gyroscope white noise
    double accel_noise_density;     ///< [m/s²/√Hz] Accelerometer white noise
    double gyro_random_walk;        ///< [rad/s²/√Hz] Gyroscope bias random walk
    double accel_random_walk;       ///< [m/s³/√Hz] Accelerometer bias random walk

    // Default: Consumer-grade MEMS (BMI088-like)
    static ImuNoiseParams default_consumer() {
        return ImuNoiseParams{
            .gyro_noise_density = 0.0001,   // 0.1 mdps/√Hz
            .accel_noise_density = 0.002,   // 2 mg/√Hz
            .gyro_random_walk = 1e-6,       // Typical value
            .accel_random_walk = 2e-5       // Typical value
        };
    }
};

// ========== Preintegrated IMU Measurement ==========

/**
 * @brief Preintegrated IMU measurement (Forster et al.)
 *
 * Contains delta measurements and Jacobians for bias correction.
 * Size: ~500 bytes (stored temporarily, not in queue)
 */
struct PreintegratedImu {
    timestamp_t start_time_ns;
    timestamp_t end_time_ns;
    double dt;                  ///< Total integration time [s]

    // Preintegrated quantities (in frame at start_time)
    Quaterniond delta_R;        ///< Rotation delta
    Vector3d delta_v;           ///< Velocity delta [m/s]
    Vector3d delta_p;           ///< Position delta [m]

    // Jacobians for bias correction (∂ΔR/∂bg, etc.)
    Matrix3d dR_dbg;
    Matrix3d dv_dbg;
    Matrix3d dv_dba;
    Matrix3d dp_dbg;
    Matrix3d dp_dba;

    // Covariance of delta measurement (9×9)
    Eigen::Matrix<double, 9, 9> covariance;

    // Noise parameters (needed for EKF covariance propagation)
    ImuNoiseParams noise_params;

    PreintegratedImu()
        : start_time_ns(0), end_time_ns(0), dt(0.0),
          delta_R(Quaterniond::Identity()),
          delta_v(Vector3d::Zero()), delta_p(Vector3d::Zero()),
          dR_dbg(Matrix3d::Identity()), dv_dbg(Matrix3d::Zero()),
          dv_dba(Matrix3d::Identity()), dp_dbg(Matrix3d::Zero()),
          dp_dba(Matrix3d::Identity()),
          covariance(Eigen::Matrix<double, 9, 9>::Zero()),
          noise_params(ImuNoiseParams::default_consumer()) {}
};

} // namespace fusion

#endif // FUSION_CORE_SENSOR_TYPES_HPP
