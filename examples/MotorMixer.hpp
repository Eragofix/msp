// MotorMixer.hpp - lightweight motor mixing component for roll axis
#ifndef MOTOR_MIXER_HPP
#define MOTOR_MIXER_HPP

#include <array>
#include <cstdint>

struct MotorCommand {
    uint16_t motor1;
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
};

class MotorMixer {
public:
    MotorMixer(double base_throttle = 1200.0,
               double min_pwm = 1100.0,
               double max_pwm = 1800.0)
        : base_throttle_(base_throttle), min_pwm_(min_pwm), max_pwm_(max_pwm) {}

    // Compute per-motor PWM values for a roll-only control signal.
    // control is the signed PWM delta to apply (+ => increase right motors)
    MotorCommand mix(double control) {
        last_input_control_ = control;

        const double m1 = base_throttle_ - control;
        const double m2 = base_throttle_ - control;
        const double m3 = base_throttle_ + control;
        const double m4 = base_throttle_ + control;

        MotorCommand out{
            clamp(m1), clamp(m2), clamp(m3), clamp(m4)
        };

        last_output_ = out;
        return out;
    }

    void setBaseThrottle(double base) { base_throttle_ = base; }
    double getBaseThrottle() const { return base_throttle_; }

    // Expose internals for logging/debug
    double getLastInputControl() const { return last_input_control_; }
    MotorCommand getLastOutput() const { return last_output_; }

private:
    uint16_t clamp(double v) const {
        if (v < min_pwm_) v = min_pwm_;
        if (v > max_pwm_) v = max_pwm_;
        return static_cast<uint16_t>(v);
    }

    double base_throttle_;
    double min_pwm_;
    double max_pwm_;

    // last values (for logging)
    double last_input_control_ = 0.0;
    MotorCommand last_output_ = {0,0,0,0};
};

#endif // MOTOR_MIXER_HPP
