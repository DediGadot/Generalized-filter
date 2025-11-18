// Minimal test to isolate infinite loop bug
#include "filter/ekf_state.hpp"
#include "filter/imu_preintegration.hpp"
#include "filter/mag_update.hpp"
#include "core/sensor_types.hpp"

#include <iostream>

using namespace fusion;

int main() {
    std::cout << "Testing minimal loop..." << std::endl;

    EkfState state;
    state.initialize(Vector3d::Zero(), Vector3d::Zero(), Quaterniond::Identity(), 0);
    state.set_initial_covariance(0.5, 0.5, 1.0, 0.001, 0.01);

    MagUpdate mag_update;
    mag_update.set_location(37.7749, -122.4194, 100.0);
    Vector3d mag_reference = mag_update.get_reference_field();

    Vector3d gravity(0, 0, 9.81);
    ImuPreintegration preint;

    std::cout << "Starting loop for 100 iterations..." << std::endl;

    for (int i = 0; i < 100; i++) {
        if (i % 10 == 0) {
            std::cout << "  i = " << i << std::endl;
        }

        // Create IMU sample
        ImuSample sample;
        sample.timestamp_ns = i * 5000000;
        sample.gyro = Vector3f::Zero();
        sample.accel = Vector3f(0, 0, -9.81);

        preint.integrate(sample);

        // EKF prediction every 20 samples
        if (i > 0 && i % 20 == 0) {
            auto preint_result = preint.get_result();
            state.predict(preint_result, gravity);
            preint.reset();
        }

        // Magnetometer update every 4 samples
        if (i > 0 && i % 4 == 0) {
            bool accepted = mag_update.update(state, mag_reference, false);
            if (accepted) {
                state.inject_error(state.error_state());
                state.reset_error();
            }
        }
    }

    std::cout << "Loop completed successfully!" << std::endl;

    return 0;
}
