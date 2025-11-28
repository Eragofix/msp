#include <Client.hpp>
#include <msp_msg.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct MotorCommand {
    uint16_t motor1;
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
};

struct ControlOutput {
    double pid_roll;
    double pid_omega;
};

struct PidSignals {
    double P;
    double I;
    double D;
};

struct Measurements {
    double roll;
    double omega_roll;
};

struct RollControllerSignals {
    ControlOutput control_output;
    MotorCommand motor_command;
    PidSignals pid_roll_signals;
    PidSignals pid_omega_signals;
    Measurements measurements;
};

struct Sample {
    std::string timestamp_utc;
    RollControllerSignals roll_controller_signals;
};
// ===========================
// PID z SATURACJĄ TYLKO NA CAŁCE
// ===========================
class PID {
public:
    PID(double kp, double ki, double kd, double dt,
        double integral_min, double integral_max)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
          prev_error_(0.0), integral_(0.0),
          integral_min_(integral_min), integral_max_(integral_max) {}

    double compute(double setpoint, double measured) {
        const double error = setpoint - measured;

        integral_ += error * dt_;
        if (integral_ > integral_max_) integral_ = integral_max_;
        if (integral_ < integral_min_) integral_ = integral_min_;

        const double derivative = (error - prev_error_) / dt_;
        prev_error_ = error;

        pid_signals_.P = kp_ * error;
        pid_signals_.I = ki_ * integral_;
        pid_signals_.D = kd_ * derivative;
    
        return pid_signals_.P + pid_signals_.I + pid_signals_.D;
    }

    PidSignals getPidSignals() const {
        return pid_signals_;
    }

private:
    double kp_, ki_, kd_, dt_;
    PidSignals pid_signals_;
    double prev_error_, integral_;
    double integral_min_, integral_max_;
};

// ===========================
// Klasa: AttitudeSensor (ROLL)
// ===========================
class AttitudeSensor {
public:
    AttitudeSensor(msp::client::Client& client, msp::FirmwareVariant fw_variant)
        : client_(client), attitude_msg_(fw_variant), roll_rad_(0.0) {}

    bool update() {
        if (client_.sendMessage(attitude_msg_) != 1) {
            std::cerr << "[AttitudeSensor] Failed to read attitude\n";
            return false;
        }
        double roll_deg = static_cast<double>(attitude_msg_.roll);
        roll_rad_ = roll_deg * M_PI / 180.0;
        return true;
    }

    double getRollRad() const { return roll_rad_; }

private:
    msp::client::Client& client_;
    msp::msg::Attitude attitude_msg_;
    double roll_rad_;
};

// ===========================
// Klasa: AngularRateSensor (RAW_IMU)
// ===========================
class AngularRateSensor {
public:
    AngularRateSensor(msp::client::Client& client, msp::FirmwareVariant fw_variant)
        : client_(client), raw_imu_msg_(fw_variant),
          omega_roll_rad_(0.0), omega_pitch_rad_(0.0), omega_yaw_rad_(0.0) {}

    bool update() {
        if (client_.sendMessage(raw_imu_msg_) != 1) {
            std::cerr << "[AngularRateSensor] Failed to read RawImu\n";
            return false;
        }

        constexpr double DEG2RAD = M_PI / 180.0;
        constexpr double GYRO_LSB_PER_DPS = 16.4;
        constexpr double GYRO_SCALE_RAD_PER_LSB = (1.0 / GYRO_LSB_PER_DPS) * DEG2RAD;

        omega_roll_rad_  = static_cast<double>(raw_imu_msg_.gyro[0]) * GYRO_SCALE_RAD_PER_LSB;
        omega_pitch_rad_ = static_cast<double>(raw_imu_msg_.gyro[1]) * GYRO_SCALE_RAD_PER_LSB;
        omega_yaw_rad_   = static_cast<double>(raw_imu_msg_.gyro[2]) * GYRO_SCALE_RAD_PER_LSB;

        return true;
    }

    double getOmegaRollRad() const { return omega_roll_rad_; }

private:
    msp::client::Client& client_;
    msp::msg::RawImu raw_imu_msg_;
    double omega_roll_rad_;
    double omega_pitch_rad_;
    double omega_yaw_rad_;
};

// ===========================
// Klasa: RollController (steruje silnikami 1-4)
// ===========================
class RollController {
public:
    RollController(msp::client::Client& client,
                   msp::FirmwareVariant fw_variant,
                   double dt,
                   double base_throttle)
        : client_(client),
        //   pid_roll_(7.2, 0.00, 0.0, dt, -400.0, 400.0), // 0 deg
        //   pid_omega_(38.5, 0.000, 0.028, dt, -100.0, 100.0), // 0 deg
          pid_roll_(7.2, 0.00, 0.0, dt, -400.0, 400.0), 
          pid_omega_(38.5, 0.000, 0.028, dt, -100.0, 100.0),
          motor_msg_(fw_variant),
          base_throttle_(base_throttle) {}

    void setRollSetpoint(double roll_setpoint_rad) { roll_setpoint_rad_ = roll_setpoint_rad; }

    void update(double roll_meas_rad, double omega_roll_meas_rad) {
        double omega_setpoint = pid_roll_.compute(roll_setpoint_rad_, roll_meas_rad);
        double control = pid_omega_.compute(omega_setpoint, omega_roll_meas_rad);

        // Silniki dla osi ROLL:
        //  - Roll w prawo → 3 i 4 ↑, 1 i 2 ↓
        //  - Roll w lewo  → 1 i 2 ↑, 3 i 4 ↓
        const double m1 = base_throttle_ - control;
        const double m2 = base_throttle_ - control;
        const double m3 = base_throttle_ + control;
        const double m4 = base_throttle_ + control;

        motor_msg_.motor = {{
            clampMotor(m1),
            clampMotor(m2),
            clampMotor(m3),
            clampMotor(m4),
            0, 0, 0, 0
        }};

        std::cout << "[RollController] control=" << control
              << " m1=" << m1
              << " m2=" << m2
              << " m3=" << m3
              << " m4=" << m4 << '\n';

        if (client_.sendMessage(motor_msg_) != 1)
            std::cerr << "[RollController] Failed to send motor command\n";

        // Zapis sygnałów do struktury RollControllerSignals
        roll_controller_signals_.control_output.pid_roll = omega_setpoint;
        roll_controller_signals_.control_output.pid_omega = control;
        roll_controller_signals_.motor_command.motor1 = clampMotor(m1);
        roll_controller_signals_.motor_command.motor2 = clampMotor(m2);
        roll_controller_signals_.motor_command.motor3 = clampMotor(m3);
        roll_controller_signals_.motor_command.motor4 = clampMotor(m4);
        roll_controller_signals_.pid_roll_signals = pid_roll_.getPidSignals();
        roll_controller_signals_.pid_omega_signals = pid_omega_.getPidSignals();
        roll_controller_signals_.measurements.roll = roll_meas_rad;
        roll_controller_signals_.measurements.omega_roll = omega_roll_meas_rad;
    }

    RollControllerSignals getRollControllerSignals() const {
        return roll_controller_signals_;
    }


private:
    static uint16_t clampMotor(double value) {
        const double min_pwm = 1100.0;
        const double max_pwm = 1800.0;
        if (value < min_pwm) value = min_pwm;
        if (value > max_pwm) value = max_pwm;
        return static_cast<uint16_t>(value);
    }

    msp::client::Client& client_;
    PID pid_roll_;
    PID pid_omega_;
    RollControllerSignals roll_controller_signals_;
    msp::msg::SetMotor motor_msg_;
    double base_throttle_;
    double roll_setpoint_rad_ = 0.0;
};


static void sendEmergencyStop(msp::client::Client& cl, msp::FirmwareVariant var) {
    msp::msg::SetMotor msg(var);
    for (size_t i = 0; i < 4; ++i) msg.motor[i] = static_cast<uint16_t>(1000);
    for (size_t i = 4; i < 8; ++i) msg.motor[i] = static_cast<uint16_t>(0);

    if (cl.sendMessage(msg) == 1)
        std::cerr << "Motors stopped\n";
    else
        std::cerr << "Failed to stop motors\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

std::atomic<bool> interrupted{false};

void signalHandler(int) {
    interrupted = true;
}

// ===========================
// main()
// ===========================
int main(int argc, char* argv[]) {

    std::signal(SIGINT, signalHandler);

    RollControllerSignals signals;

    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    msp::client::Client  client;
    client.setLoggingLevel(msp::client::LoggingLevel::WARNING);
    client.setVariant(msp::FirmwareVariant::BAFL);
    client.start(device, baudrate);

    msp::FirmwareVariant fw_variant = msp::FirmwareVariant::BAFL;

    const double dt = 0.001;
    const int loop_sleep_ms = static_cast<int>(dt * 1000.0);

    AttitudeSensor attitude_sensor(client, fw_variant);
    AngularRateSensor rate_sensor(client, fw_variant);
    RollController roll_controller(client, fw_variant, dt, 1200.0);

    roll_controller.setRollSetpoint(0.0); // poziom = 0 rad

    while (!interrupted) {
        if (!attitude_sensor.update() || !rate_sensor.update()) continue;

        double roll_rad = attitude_sensor.getRollRad();
        double omega_roll_rad = rate_sensor.getOmegaRollRad();

        std::cout << "ROLL = " << roll_rad
                    << " rad,  OMEGA = " << omega_roll_rad
                    << " rad/s\n";

        roll_controller.update(roll_rad, omega_roll_rad);
        signals = roll_controller.getRollControllerSignals();
        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }
    std::cerr << "Control loop interrupted: " << std::endl;

    sendEmergencyStop(client, fw_variant);
    client.stop();
    std::cout << "PROGRAM COMPLETE\n";
    return 0;
}
