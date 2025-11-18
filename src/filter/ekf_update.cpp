// Generic EKF Measurement Update Implementation
#include "filter/ekf_update.hpp"

namespace fusion {

MeasurementUpdate::MeasurementUpdate()
    : num_updates_(0), num_outliers_(0) {}

double MeasurementUpdate::get_chi_square_threshold(int dof, double confidence_level) {
    // Chi-square critical values for common cases
    // Source: Chi-square distribution table
    //
    // These values represent the threshold where P(X² < threshold) = confidence_level
    // If the Mahalanobis distance exceeds this, reject as outlier

    // 95% confidence level
    if (confidence_level >= 0.94 && confidence_level <= 0.96) {
        switch (dof) {
            case 1: return 3.841;
            case 2: return 5.991;
            case 3: return 7.815;
            case 4: return 9.488;
            case 5: return 11.070;
            case 6: return 12.592;
            default: return 7.815;  // Default to 3 DOF
        }
    }

    // 99% confidence level
    if (confidence_level >= 0.98 && confidence_level <= 1.0) {
        switch (dof) {
            case 1: return 6.635;
            case 2: return 9.210;
            case 3: return 11.345;
            case 4: return 13.277;
            case 5: return 15.086;
            case 6: return 16.812;
            default: return 11.345;  // Default to 3 DOF
        }
    }

    // Default: 95% confidence, 3 DOF (position measurements)
    return 7.815;
}

}  // namespace fusion
