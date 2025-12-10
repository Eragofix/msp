#include <Client.hpp>
#include <msp_msg.hpp>

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

struct MotorCommand {
    uint16_t motor1;
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
};

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
    std::time_t now_c = system_clock::to_time_t(now);
    std::tm* tm_utc = std::gmtime(&now_c); // Linux/RPi

    std::ostringstream oss;
    oss << std::put_time(tm_utc, "%Y-%m-%d %H:%M:%S");
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

    // zmiana Kp w locie
    void setKp(double kp) { kp_ = kp; }

private:
    double kp_, ki_, kd_, dt_;
    PidSignals pid_signals_{};
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
          pid_roll_(8.5, 10, 0.0, dt, -400.0, 400.0),
          pid_omega_(20.0, 0.000, 0.001, dt, -100.0, 100.0),
          roll_controller_signals_(),
          motor_msg_(fw_variant),
          base_throttle_(base_throttle),
          roll_setpoint_rad_(0.0),
          kp_roll_base_(8.5),   // bazowe Kp roll
          kp_omega_base_(20.0)  // bazowe Kp omega
    {}

    void setRollSetpoint(double roll_setpoint_rad) { roll_setpoint_rad_ = roll_setpoint_rad; }

    void update(double roll_meas_rad, double omega_roll_meas_rad) {
        // wspólna skala zależna od |roll|
        const double abs_roll = std::fabs(roll_meas_rad);
        const double roll_for_max_kp_rad = 20.0 * M_PI / 180.0; // 20 stopni

        double rel = abs_roll / roll_for_max_kp_rad; // 0..~∞
        if (rel > 1.0) rel = 1.0;                    // saturacja do [0,1]

        // skala 0.4..1.0
        const double scale = 0.4 + 0.6 * rel;

        // === dynamiczne Kp dla PID po kącie (roll) ===
        const double kp_roll_dynamic = kp_roll_base_ * scale;
        pid_roll_.setKp(kp_roll_dynamic);

        // 1) PID po kącie
        double omega_setpoint = pid_roll_.compute(roll_setpoint_rad_, roll_meas_rad);

        // === dynamiczne Kp dla PID po prędkości (omega) ===
        const double kp_omega_dynamic = kp_omega_base_ * scale;
        pid_omega_.setKp(kp_omega_dynamic);

        // 2) PID po prędkości z dynamicznym Kp
        double control = pid_omega_.compute(omega_setpoint, omega_roll_meas_rad); 

        // Silniki dla osi ROLL:
        //  - Roll w prawo → 3 i 4 ↑, 1 i 2 ↓
        //  - Roll w lewo  → 1 i 2 ↑, 3 i 4 ↓
        const double m1 = base_throttle_ - control;
        const double m2 = base_throttle_ - control;
        const double m3 = base_throttle_ + control;
        const double m4 = base_throttle_ + control;

        const uint16_t m1_pwm = clampMotor(m1);
        const uint16_t m2_pwm = clampMotor(m2);
        const uint16_t m3_pwm = clampMotor(m3);
        const uint16_t m4_pwm = clampMotor(m4);

        motor_msg_.motor = {{
            m1_pwm,
            m2_pwm,
            m3_pwm,
            m4_pwm,
            0, 0, 0, 0
        }};

        std::cout << "[RollController] control=" << control
                  << " Kp_roll=" << kp_roll_dynamic
                  << " Kp_omega=" << kp_omega_dynamic
                  << " roll=" << roll_meas_rad
                  << " m1=" << m1
                  << " m2=" << m2
                  << " m3=" << m3
                  << " m4=" << m4 << '\n';

        if (client_.sendMessage(motor_msg_) != 1)
            std::cerr << "[RollController] Failed to send motor command\n";

        // Sygnały PID zapisujemy tu.
        roll_controller_signals_.control_output.pid_roll   = omega_setpoint;
        roll_controller_signals_.control_output.pid_omega  = control;
        roll_controller_signals_.motor_command.motor1      = m1_pwm;
        roll_controller_signals_.motor_command.motor2      = m2_pwm;
        roll_controller_signals_.motor_command.motor3      = m3_pwm;
        roll_controller_signals_.motor_command.motor4      = m4_pwm;
        roll_controller_signals_.pid_roll_signals          = pid_roll_.getPidSignals();
        roll_controller_signals_.pid_omega_signals         = pid_omega_.getPidSignals();
        // Pola measurements uzupełnimy w main()
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
    double roll_setpoint_rad_;

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

    const double dt = 0.001;
    const int loop_sleep_ms = static_cast<int>(dt * 1000.0);

    // === PARAMETRY FILTRÓW I WYBORU ŹRÓDŁA OMEGA ===
    const double omega_filter_alpha = 0.001;    // 0..1, im bliżej 1 tym mocniejsze wygładzenie
    const bool use_gyro_for_omega   = true;     // true -> używaj filtrowanego gyro
                                                // false -> używaj filtrowanej pochodnej kąta
    double prev_roll_rad = 0.0;
    double omega_from_roll_filt = 0.0;
    double omega_gyro_filt      = 0.0;
    bool first_sample = true;
    // ===============================================

    AttitudeSensor attitude_sensor(client, fw_variant);
    AngularRateSensor rate_sensor(client, fw_variant);
    RollController roll_controller(client, fw_variant, dt, 1200.0);

    roll_controller.setRollSetpoint(0.0); // poziom = 0 rad

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

        if (first_sample) {
            prev_roll_rad = roll_rad;
            omega_from_roll_filt = 0.0;
            omega_gyro_filt = omega_gyro_raw;
            first_sample = false;
        }

        // Surowa pochodna kąta
        double omega_from_roll_raw = (roll_rad - prev_roll_rad) / dt;
        prev_roll_rad = roll_rad;

        // Filtrowanie (prosta 1. rzędu: y_k = α y_{k-1} + (1-α) x_k)
        omega_from_roll_filt =
            omega_filter_alpha * omega_from_roll_filt +
            (1.0 - omega_filter_alpha) * omega_from_roll_raw;

        omega_gyro_filt =
            omega_filter_alpha * omega_gyro_filt +
            (1.0 - omega_filter_alpha) * omega_gyro_raw;

        // Wybór sygnału do regulatora
        double omega_for_controller = use_gyro_for_omega
                                      ? omega_gyro_filt
                                      : omega_from_roll_filt;

        std::cout << "ROLL = " << roll_rad
                  << " rad,  OMEGA_used = " << omega_for_controller
                  << " rad/s\n";

        roll_controller.update(roll_rad, omega_for_controller);
        signals = roll_controller.getRollControllerSignals();

        // UZUPEŁNIENIE PÓL MEASUREMENTS (NA POTRZEBY LOGÓW)
        signals.measurements.roll               = roll_rad;
        signals.measurements.omega_roll         = omega_for_controller;
        signals.measurements.omega_roll_from_roll = omega_from_roll_filt;
        signals.measurements.omega_roll_gyro      = omega_gyro_raw;
        signals.measurements.omega_roll_gyro_filt = omega_gyro_filt;

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
