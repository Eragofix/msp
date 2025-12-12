// Lightweight filtering utilities used by examples
// Provides a 1st-order low-pass filter and a small helper that computes
// an angular-rate estimate either from gyro or from the filtered derivative
// of the attitude angle.
#pragma once

#include <cmath>

class LowPassFilter {
public:
    LowPassFilter(double alpha = 0.5, double init = 0.0)
        : alpha_(alpha), y_(init), initialized_(false) {}

    // Update the filter with new input x and return the filtered value.
    // Implements: y_k = alpha * y_{k-1} + (1-alpha) * x_k
    double update(double x) {
        if (!initialized_) {
            y_ = x;
            initialized_ = true;
            return y_;
        }
        y_ = alpha_ * y_ + (1.0 - alpha_) * x;
        return y_;
    }

    double value() const { return y_; }
    void setAlpha(double a) { alpha_ = a; }

private:
    double alpha_;
    double y_;
    bool initialized_;
};

// AngularRateFilter encapsulates the small amount of logic used in
// `test_attitude.cpp` to (a) compute a raw derivative of the roll angle,
// (b) low-pass filter that derivative, (c) low-pass filter the gyro and
// (d) pick which signal to feed to the controller.
class AngularRateFilter {
public:
    AngularRateFilter(double dt, double alpha, bool useGyro)
        : dt_(dt), lpf_deriv_(alpha, 0.0), lpf_gyro_(alpha, 0.0),
          use_gyro_(useGyro), prev_angle_(0.0), first_(true),
          omega_from_roll_filt_(0.0), omega_gyro_filt_(0.0),
          omega_for_controller_(0.0), omega_from_roll_raw_(0.0) {}

    // Call every loop with the latest roll angle (rad) and raw gyro (rad/s).
    // Returns the selected omega value to be used by the controller.
    double update(double roll_rad, double gyro_raw) {
        if (first_) {
            prev_angle_ = roll_rad;
            lpf_deriv_.update(0.0); // initialize internal state
            lpf_gyro_.update(gyro_raw);
            first_ = false;
            omega_from_roll_raw_ = 0.0;
        }

        double omega_from_roll_raw = (roll_rad - prev_angle_) / dt_;
        prev_angle_ = roll_rad;
        omega_from_roll_raw_ = omega_from_roll_raw;

        omega_from_roll_filt_ = lpf_deriv_.update(omega_from_roll_raw);
        omega_gyro_filt_ = lpf_gyro_.update(gyro_raw);

        omega_for_controller_ = use_gyro_ ? omega_gyro_filt_ : omega_from_roll_filt_;
        return omega_for_controller_;
    }

    double getOmegaForController() const { return omega_for_controller_; }
    double getOmegaFromRollFilt() const { return omega_from_roll_filt_; }
    double getOmegaGyroFilt() const { return omega_gyro_filt_; }
    double getOmegaFromRollRaw() const { return omega_from_roll_raw_; }

    void setUseGyro(bool use) { use_gyro_ = use; }
    void setAlpha(double alpha) { lpf_deriv_.setAlpha(alpha); lpf_gyro_.setAlpha(alpha); }

private:
    double dt_;
    LowPassFilter lpf_deriv_;
    LowPassFilter lpf_gyro_;
    bool use_gyro_;
    double prev_angle_;
    bool first_;
    double omega_from_roll_filt_;
    double omega_gyro_filt_;
    double omega_for_controller_;
    double omega_from_roll_raw_;
};
