#include <Client.hpp>
#include <msp_msg.hpp>
#include "MotorMixer.hpp"
#include "MotorSender.hpp"
#include "Filter.hpp"
#include "AttitudeSensor.hpp"
#include "AngularRateSensor.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <atomic>

// ==== DODANE DO LOGOWANIA ====
#include <fstream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>
// =============================

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fs = std::filesystem;

// MotorCommand is provided by MotorMixer.hpp

struct ControlOutput {
    double pid_roll;   // sygnał z regulatora kąta (ω_setpoint)
    double pid_omega;  // sygnał z regulatora prędkości (moment/Δthrottle)
};

struct PidSignals {
    double P;
    double I;
    double D;
};

// =======================
// Pomiar / estymaty
// =======================
//
// roll                – kąt (rad)
// omega_roll          – sygnał użyty w regulatorze (w zależności od trybu)
// omega_roll_from_roll – filtrowana pochodna kąta
// omega_roll_gyro      – surowy gyro (jak było)
// omega_roll_gyro_filt – filtrowany gyro
//
struct Measurements {
    double roll;
    double omega_roll;
    double omega_roll_from_roll;
    double omega_roll_gyro;
    double omega_roll_gyro_filt;
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

// ========== STRUKTURY I ZMIENNE DO LOGOWANIA ==========
std::queue<Sample> log_queue;
std::mutex log_mutex;
std::condition_variable log_cv;
std::atomic<bool> logging_finished{false};
// ======================================================

// Timestamp UTC do logu: YYYY-MM-DD HH:MM:SS
std::string currentUtcTimestamp() {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto now_us = time_point_cast<microseconds>(now);
    auto us = now_us.time_since_epoch() % seconds(1);

    std::time_t now_c = system_clock::to_time_t(now);
    std::tm* tm_utc = std::gmtime(&now_c); // Linux/RPi

    std::ostringstream oss;
    oss << std::put_time(tm_utc, "%Y-%m-%d_%H:%M:%S")
        << '.' << std::setw(6) << std::setfill('0') << us.count();

    return oss.str();
}


// Timestamp do nazwy pliku: YYYYMMDD_HHMMSS
std::string currentUtcTimestampForFilename() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t now_c = system_clock::to_time_t(now);
    std::tm* tm_utc = std::gmtime(&now_c);

    std::ostringstream oss;
    oss << std::put_time(tm_utc, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ===========================
// PID z SATURACJĄ TYLKO NA CAŁCE
// + możliwość zmiany Kp w locie
// ===========================
class PID {
public:
    PID(double kp, double ki, double kd, double dt,
        double integral_min, double integral_max,
        bool deadzone_flag, bool lowpass_filter_flag)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
          prev_error_(0.0), integral_(0.0),
          integral_min_(integral_min), integral_max_(integral_max),
          deadzone_flag_(deadzone_flag),
          lowpass_filter_flag_(lowpass_filter_flag),
          d_state_(0.0)
    {}

    double compute(double setpoint, double measured) {
        const double error = setpoint - measured;

        // Deadzone tylko dla P i I
        const double ePI = deadzone_flag_ ? deadzone(error) : error;

        // I liczona z ePI (deadzone wpływa na całkę) + saturacja tylko na całce
        integral_ += ePI * dt_;
        if (integral_ > integral_max_) integral_ = integral_max_;
        if (integral_ < integral_min_) integral_ = integral_min_;

        // Surowa pochodna z error (bez deadzone)
        const double derivative_raw = (error - prev_error_) / dt_;
        prev_error_ = error;

        // D z filtrem dolnoprzepustowym 1/(50s+1) tylko jeśli flaga włączona
        double derivative_used = derivative_raw;
        if (lowpass_filter_flag_) {
            const double tau = 50;                 // 1/(50s+1)
            const double a = std::exp(-dt_ / tau);    // dyskretny LPF 1 rzędu
            d_state_ = a * d_state_ + (1.0 - a) * derivative_raw;
            derivative_used = d_state_;
        }

        pid_signals_.P = kp_ * ePI;
        pid_signals_.I = ki_ * integral_;
        pid_signals_.D = kd_ * derivative_used;

        return pid_signals_.P + pid_signals_.I + pid_signals_.D;
    }

    PidSignals getPidSignals() const {
        return pid_signals_;
    }

    // zmiana Kp w locie
    void setKp(double kp) { kp_ = kp; }

private:
    static double deadzone(double x) {
        constexpr double start = -0.052;
        constexpr double end   =  0.052;
        if (x >= start && x <= end) return 0.0;
        if (x > end)  return x - end;
        return x - start;
    }

    double kp_, ki_, kd_, dt_;
    PidSignals pid_signals_{};
    double prev_error_, integral_;
    double integral_min_, integral_max_;
    bool deadzone_flag_;
    bool lowpass_filter_flag_;

    double d_state_; // stan filtra D (tylko gdy lowpass_filter_flag_==true)
};


// AttitudeSensor and AngularRateSensor have been moved to separate files
// AttitudeSensor: examples/AttitudeSensor.hpp/cpp
// AngularRateSensor: examples/AngularRateSensor.hpp/cpp


class GravityCompensator {
    public:
        GravityCompensator(double a_, double b_, double c_,
                           double m_, double g_, double l_cog_, double l_,
                           double base_throttle = 1200.0)
            : a(a_), b(b_), c(c_), m(m_), g(g_), l_cog(l_cog_), l(l_)
        {
            updateKt(base_throttle);
            updateKff();
        }

        // Update linearized thrust gain k_T based on current base throttle (PWM)
        void updateKt(double base_throttle) {
            Kt_ = 2.0 * a * base_throttle + b;
        }

        // Recompute feedforward gain Kff (must be called after updateKt)
        void updateKff() {
            // protect against division by zero
            if (Kt_ == 0.0) Kff_ = 0.0;
            else Kff_ = (m * g * l_cog) / (4.0 * l * Kt_);
        }

        // Compute gravity feedforward contribution in PWM for given roll (rad)
        // base_throttle is used to recompute linearized k_T if it changed
        double feedforwardPWM(double roll_rad, double base_throttle) {
            updateKt(base_throttle);
            updateKff();
            return Kff_ * std::sin(roll_rad);
        }

        // Convenience: return total control (controller PWM + gravity FF)
        double apply(double controller_pwm, double roll_rad, double base_throttle) {
            return controller_pwm + feedforwardPWM(roll_rad, base_throttle);
        }

        // Accessors
        double Kt() const { return Kt_; }
        double Kff() const { return Kff_; }
    private:
        double Kt_; // delta_PWM to delta_thrust gain  
        double Kff_; // feedforward gain
        double a, b, c;
        double m, g, l_cog, l;
};


// ===========================
// Klasa: RollController (steruje silnikami 1-4)
// ===========================
class RollController {
public:
    RollController(msp::client::Client& client,
             msp::FirmwareVariant /*fw_variant*/,
             double dt,
             double base_throttle)
      : client_(client),
        pid_roll_(5.0, 5.0, 0.0, dt, -400.0, 400.0, false, false),
        pid_omega_(30.0, 0.000, 0.028, dt, -100.0, 100.0, true, true),
        roll_controller_signals_(),
        base_throttle_(base_throttle),
        roll_setpoint_rad_(0.0),
        kp_roll_base_(8.5),   // bazowe Kp roll
        kp_omega_base_(20.0)  // bazowe Kp omega
    {}

    void setRollSetpoint(double roll_setpoint_rad) { roll_setpoint_rad_ = roll_setpoint_rad; }

    

    ControlOutput calculate(double roll_meas_rad, double omega_roll_meas_rad) {
        // wspólna skala zależna od |roll|
        // const double abs_roll = std::fabs(roll_meas_rad);
        // const double roll_for_max_kp_rad = 20.0 * M_PI / 180.0; // 20 stopni

        // double rel = abs_roll / roll_for_max_kp_rad; // 0..~∞
        // if (rel > 1.0) rel = 1.0;                    // saturacja do [0,1]

        // skala 0.4..1.0
        // const double scale = 0.4 + 0.6 * rel;

        // === dynamiczne Kp dla PID po kącie (roll) ===
        // const double kp_roll_dynamic = kp_roll_base_ * scale;
        // pid_roll_.setKp(kp_roll_dynamic);

        // 1) PID po kącie
        double omega_setpoint = pid_roll_.compute(roll_setpoint_rad_, roll_meas_rad);

        // === dynamiczne Kp dla PID po prędkości (omega) ===
        // const double kp_omega_dynamic = kp_omega_base_ * scale;
        // pid_omega_.setKp(kp_omega_dynamic);

        // 2) PID po prędkości z dynamicznym Kp
        double control = pid_omega_.compute(omega_setpoint, omega_roll_meas_rad); 

        // Sygnały PID zapisujemy wewnętrznie (dla logów). The controller
        // returns control explicitly to the caller; mixing/sending remain
        // the caller's responsibility.
        roll_controller_signals_.control_output.pid_roll   = omega_setpoint;
        roll_controller_signals_.control_output.pid_omega  = control;
        roll_controller_signals_.pid_roll_signals          = pid_roll_.getPidSignals();
        roll_controller_signals_.pid_omega_signals         = pid_omega_.getPidSignals();
        // Pola measurements uzupełnimy w main() as before.

        ControlOutput out;
        out.pid_roll = omega_setpoint;
        out.pid_omega = control;
        return out;
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
    double base_throttle_;
    double roll_setpoint_rad_;
    // mixing and sender components are handled outside the controller

    // bazowe Kp dla PID_roll i PID_omega
    double kp_roll_base_;
    double kp_omega_base_;
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

    RollControllerSignals signals{};

    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    msp::client::Client  client;
    client.setLoggingLevel(msp::client::LoggingLevel::WARNING);
    client.setVariant(msp::FirmwareVariant::BAFL);
    client.start(device, baudrate);

    msp::FirmwareVariant fw_variant = msp::FirmwareVariant::BAFL;

    const double dt = 0.0001;
    const int loop_sleep_ms = static_cast<int>(dt * 1000.0);
    const double PWM_0 = 1400.0; //  bazowy sygnał PWM

    // control settings 
    // bool compensation_enabled = true;
    // // copter model parameters
    // const double m = ;
    // const double g = 9.81;
    // const double l = ;
    // const double l_cog = ;

    // // pwm -> throttle model parameters (a*PWM^2 + b*PWM + c)
    // const double a = ;
    // const double b = ;
    // const double c = ;
    // // linearized pwm -> throttle model
    // const double kt = 2.0 * a * PWM_0 + b;

    // // compensation component 
    // double Kff = (m * g * l_cog) / (4.0 * l * kt);
    // double gravity_ff_pwm = 0.0;

    // === PARAMETRY FILTRÓW I WYBORU ŹRÓDŁA OMEGA ===
    const double omega_filter_alpha = 0.00  ;    // 0..1, im bliżej 1 tym mocniejsze wygładzenie
    const bool use_gyro_for_omega   = true; // false is dangerous     // true -> używaj filtrowanego gyro
                                                // false -> używaj filtrowanej pochodnej kąta
    // Use AngularRateFilter to encapsulate derivative + LPF logic
    AngularRateFilter omega_filter(dt, omega_filter_alpha, use_gyro_for_omega);
    // ===============================================

    AttitudeSensor attitude_sensor(client, fw_variant);
    AngularRateSensor rate_sensor(client, fw_variant);
    RollController roll_controller(client, fw_variant, dt, PWM_0);

    roll_controller.setRollSetpoint(-0.3); // poziom = 0 rad

    // External mixing and sending components (moved out of RollController)
    MotorMixer mixer(PWM_0);
    MotorSender sender(client, fw_variant);

    // ====== OTWARCIE PLIKU I START LOGGERA ======

    try {
        fs::create_directories("logs");
    } catch (const std::exception& e) {
        std::cerr << "Nie udalo sie utworzyc katalogu logs: " << e.what() << "\n";
    }

    const std::string log_filename =
        "logs/log_roll_signals_" + currentUtcTimestampForFilename() + ".csv";

    std::ofstream log_file(log_filename);
    if (!log_file.is_open()) {
        std::cerr << "Nie mozna otworzyc pliku " << log_filename << "!\n";
    } else {
        std::cout << "Logowanie do pliku: " << log_filename << "\n";
    }

    if (log_file.is_open()) {
        log_file << std::fixed << std::setprecision(6);

        // nagłówek CSV pod Excela (separator ;)
        log_file << "timestamp_utc;"
                 << "roll_rad;"
                 << "omega_used_rad_s;"
                 << "omega_from_roll_filt_rad_s;"
                 << "omega_gyro_raw_rad_s;"
                 << "omega_gyro_filt_rad_s;"
                 << "u_roll;"
                 << "u_omega;"
                 << "motor1;"
                 << "motor2;"
                 << "motor3;"
                 << "motor4;"
                 << "pid_roll_P;"
                 << "pid_roll_I;"
                 << "pid_roll_D;"
                 << "pid_omega_P;"
                 << "pid_omega_I;"
                 << "pid_omega_D;"
                 << '\n';
    }

    std::thread logger_thread([&]() {
        if (!log_file.is_open()) return;

        while (!logging_finished || !log_queue.empty()) {
            std::unique_lock<std::mutex> lk(log_mutex);
            log_cv.wait(lk, [] {
                return logging_finished || !log_queue.empty();
            });

            while (!log_queue.empty()) {
                Sample s = log_queue.front();
                log_queue.pop();
                lk.unlock();

                const auto& r = s.roll_controller_signals;
                const auto& m = r.measurements;

                log_file << s.timestamp_utc << ';'
                         << m.roll << ';'
                         << m.omega_roll << ';'
                         << m.omega_roll_from_roll << ';'
                         << m.omega_roll_gyro << ';'
                         << m.omega_roll_gyro_filt << ';'
                         << r.control_output.pid_roll << ';'
                         << r.control_output.pid_omega << ';'
                         << r.motor_command.motor1 << ';'
                         << r.motor_command.motor2 << ';'
                         << r.motor_command.motor3 << ';'
                         << r.motor_command.motor4 << ';'
                         << r.pid_roll_signals.P << ';'
                         << r.pid_roll_signals.I << ';'
                         << r.pid_roll_signals.D << ';'
                         << r.pid_omega_signals.P << ';'
                         << r.pid_omega_signals.I << ';'
                         << r.pid_omega_signals.D << ';'
                         << '\n';

                lk.lock();
            }
        }
    });
    // =================================================

    while (!interrupted) {
        if (!attitude_sensor.update() || !rate_sensor.update()) continue;

        double roll_rad = attitude_sensor.getRollRad();
        double omega_gyro_raw = rate_sensor.getOmegaRollRad();

        // Compute filtered angular rate using the AngularRateFilter component.
        double omega_for_controller = omega_filter.update(roll_rad, omega_gyro_raw);
        // double omega_for_controller =  omega_gyro_raw;

        std::cout << "ROLL = " << roll_rad
                  << " rad,  OMEGA_used = " << omega_for_controller
                  << " rad/s\n";

        ControlOutput control = roll_controller.calculate(roll_rad, omega_for_controller);
        // retrieve signals for logging (they are still maintained internally)
        signals = roll_controller.getRollControllerSignals();
        // expose the explicit control returned by the controller
        signals.control_output = control;

        // Perform mixing and send motor outputs using external components
        MotorCommand cmds = mixer.mix(signals.control_output.pid_omega);
        std::array<uint16_t,4> arr = {cmds.motor1, cmds.motor2, cmds.motor3, cmds.motor4};
        int send_res = sender.send(arr);

        // expose motor commands in signals for logging
        signals.motor_command.motor1 = cmds.motor1;
        signals.motor_command.motor2 = cmds.motor2;
        signals.motor_command.motor3 = cmds.motor3;
        signals.motor_command.motor4 = cmds.motor4;

        std::cout << "[Main] control=" << signals.control_output.pid_omega
            << " roll=" << roll_rad
            << " m1=" << cmds.motor1
            << " m2=" << cmds.motor2
            << " m3=" << cmds.motor3
            << " m4=" << cmds.motor4
            << " send_res=" << send_res << '\n';

        // UZUPEŁNIENIE PÓL MEASUREMENTS (NA POTRZEBY LOGÓW)
        signals.measurements.roll               = roll_rad;
        signals.measurements.omega_roll         = omega_for_controller;
        signals.measurements.omega_roll_from_roll = omega_filter.getOmegaFromRollFilt();
        signals.measurements.omega_roll_gyro      = omega_gyro_raw;
        signals.measurements.omega_roll_gyro_filt = omega_filter.getOmegaGyroFilt();

        Sample sample;
        sample.timestamp_utc = currentUtcTimestamp();
        sample.roll_controller_signals = signals;

        {
            std::lock_guard<std::mutex> lk(log_mutex);
            log_queue.push(sample);
        }
        log_cv.notify_one();

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    std::cerr << "Control loop interrupted." << std::endl;

    sendEmergencyStop(client, fw_variant);
    client.stop();

    // ====== ZAMKNIĘCIE LOGGERA ======
    logging_finished = true;
    log_cv.notify_all();
    if (logger_thread.joinable()) {
        logger_thread.join();
    }
    if (log_file.is_open()) {
        log_file.close();
    }
    // =================================

    std::cout << "PROGRAM COMPLETE\n";
    return 0;
}
