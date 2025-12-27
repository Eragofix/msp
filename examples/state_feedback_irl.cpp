#include <Client.hpp>
#include <msp_msg.hpp>
#include "MotorMixer.hpp"
#include "MotorSender.hpp"
#include "Filter.hpp"
#include "AttitudeSensor.hpp"
#include "AngularRateSensor.hpp"
#include "model_parameters.hpp"
#include "state_feedback.hpp"
#include "AsyncCsvLogger.hpp"

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

    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    // Optional CLI: phi0 wn zeta_d
    const double phi0 = (argc > 3) ? std::stod(argv[3]) : -0.0;    // rad
    const double wn = (argc > 4) ? std::stod(argv[4]) : 3.0;      // rad/s
    const double zeta_d = (argc > 5) ? std::stod(argv[5]) : 0.45;   // [-]

    msp::client::Client  client;
    client.setLoggingLevel(msp::client::LoggingLevel::WARNING);
    client.setVariant(msp::FirmwareVariant::BAFL);
    client.start(device, baudrate);

    msp::FirmwareVariant fw_variant = msp::FirmwareVariant::BAFL;

    const double dt = 0.0001;
    // use dt-based duration directly in loop sleep

    ModelParameters params;
    AttitudeSensor attitude_sensor(client, fw_variant);
    AngularRateSensor rate_sensor(client, fw_variant);
    StateFeedbackController roll_controller(
        params,
        phi0,
        0.0,    // omega_phi0 (rad/s)
        wn,
        zeta_d
    );

    const double omega_filter_alpha = 0.00;
    const bool use_gyro_for_omega   = true;
    AngularRateFilter omega_filter(dt, omega_filter_alpha, use_gyro_for_omega);

    MotorMixer mixer(params.PWM_base);
    MotorSender sender(client, fw_variant);

    // Async CSV logger
    AsyncCsvLogger logger("logs", "log_state_feedback_irl_", 4096);

    while (!interrupted) {
        if (!attitude_sensor.update() || !rate_sensor.update()) continue;
        
        double roll_rad = attitude_sensor.getRollRad();
        double omega_gyro_raw = rate_sensor.getOmegaRollRad();
        double omega_for_controller = omega_filter.update(roll_rad, omega_gyro_raw);

        // compute raw control
        double u_raw = roll_controller.compute(roll_rad, omega_for_controller);

        // Saturate control signal to [-400, 400] (keeping same limits as other examples)
        double u_sat = u_raw;
        if (u_sat > 400.0) u_sat = 400.0;
        if (u_sat < -400.0) u_sat = -400.0;

        // Fetch u0 for logging
        const double u0 = roll_controller.getU0();

        std::cout << "ROLL=" << roll_rad
                  << " rad | OMEGA=" << omega_for_controller
                  << " rad/s | u0=" << u0
                  << " | u_sat=" << u_sat << "\n";

        // Perform mixing and send motor outputs using external components
        MotorCommand cmds = mixer.mix(u_sat);
        std::array<uint16_t,4> arr = {cmds.motor1, cmds.motor2, cmds.motor3, cmds.motor4};
        int send_res = sender.send(arr);

        std::cout << "[Main] control=" << u_sat
            << " roll=" << roll_rad
            << " m1=" << cmds.motor1
            << " m2=" << cmds.motor2
            << " m3=" << cmds.motor3
            << " m4=" << cmds.motor4
            << " send_res=" << send_res << '\n';

        // Prepare and push lightweight POD row to logger
        LogRow row{};
        row.roll_rad = roll_rad;
        row.omega_used_rad_s = omega_for_controller;
        row.omega_gyro_raw_rad_s = omega_gyro_raw;
        // try to fetch gyro filtered value if available from filter
        double gyro_filt = 0.0;
#ifdef HAS_ANGULAR_RATE_FILTER_GETTERS
        gyro_filt = omega_filter.getOmegaGyroFilt();
#endif
        row.omega_gyro_filt_rad_s = gyro_filt;
        row.u_raw = u_raw;
        row.u_sat = u_sat;
        row.motor1 = cmds.motor1;
        row.motor2 = cmds.motor2;
        row.motor3 = cmds.motor3;
        row.motor4 = cmds.motor4;

        logger.push(row);
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }

    std::cerr << "Control loop interrupted." << std::endl;

    sendEmergencyStop(client, fw_variant);

    // ensure logger is stopped and flushed before exiting
    logger.stop();

    client.stop();

    std::cout << "PROGRAM COMPLETE\n";
    return 0;
}
