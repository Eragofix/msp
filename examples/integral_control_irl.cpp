#include <Client.hpp>
#include <msp_msg.hpp>
#include "MotorMixer.hpp"
#include "MotorSender.hpp"
#include "Filter.hpp"
#include "AttitudeSensor.hpp"
#include "AngularRateSensor.hpp"
#include "model_parameters.hpp"
#include "integral_state_feedback.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <atomic>
#include <array>
#include <iomanip>
#include <algorithm>

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

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);

    const std::string device = (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    // CLI: phi0 wn zeta_d pI dt
    const double phi0 = (argc > 3) ? std::stod(argv[3]) : -0.0;    // rad
    const double wn = (argc > 4) ? std::stod(argv[4]) : 2.5;      // rad/s
    const double zeta_d = (argc > 5) ? std::stod(argv[5]) : 0.45;   // [-]
    const double pI = (argc > 6) ? std::stod(argv[6]) : 1.0;       // integrator pole
    const double dt = (argc > 7) ? std::stod(argv[7]) : 0.00005; // if >0 use as dt

    msp::client::Client  client;
    client.setLoggingLevel(msp::client::LoggingLevel::WARNING);
    client.setVariant(msp::FirmwareVariant::BAFL);
    client.start(device, baudrate);

    msp::FirmwareVariant fw_variant = msp::FirmwareVariant::BAFL;

    ModelParameters params;
    AttitudeSensor attitude_sensor(client, fw_variant);
    AngularRateSensor rate_sensor(client, fw_variant);

    // Angular rate filter init (nominal dt used for internal filter state only)
    const double omega_filter_alpha = 0.00;
    const bool use_gyro_for_omega = true;
    AngularRateFilter omega_filter(dt, omega_filter_alpha, use_gyro_for_omega);

    // Integral controller instance
    IntegralStateFeedbackController ic(
        params,
        phi0,
        0.0, // omega_phi0
        wn,
        zeta_d,
        pI
    );

    MotorMixer mixer(params.PWM_base);
    MotorSender sender(client, fw_variant);

    std::cout << "IRL integral state-feedback started\n";
    std::cout << std::fixed << std::setprecision(4);
    const int loop_sleep_ms = static_cast<int>(dt * 1000.0);

    while (!interrupted) {
        if (!attitude_sensor.update() || !rate_sensor.update()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        double roll_rad = attitude_sensor.getRollRad();
        double omega_gyro_raw = rate_sensor.getOmegaRollRad();
        double omega_for_controller = omega_filter.update(roll_rad, omega_gyro_raw);

        double u_raw = ic.compute(roll_rad, omega_for_controller, dt);
        double u_sat = std::clamp(u_raw, -400.0, 400.0);

        MotorCommand cmds = mixer.mix(u_sat);
        std::array<uint16_t,4> arr = {cmds.motor1, cmds.motor2, cmds.motor3, cmds.motor4};
        int send_res = sender.send(arr);

        std::cout << "ROLL=" << roll_rad
                  << " rad, OMEGA_used=" << omega_for_controller
                  << " rad/s, dt=" << dt
                  << " | u_raw=" << u_raw
                  << " | u_sat=" << u_sat
                  << " | u0=" << ic.getU0()
                  << " | k1=" << ic.getK1()
                  << " | k2=" << ic.getK2()
                  << " | Fi=" << ic.getFi()
                  << " | xi=" << ic.getXi()
                  << " send_res=" << send_res
                  << '\n';

        std::this_thread::sleep_for(std::chrono::milliseconds(loop_sleep_ms));
    }

    std::cerr << "Control loop interrupted." << std::endl;

    sendEmergencyStop(client, fw_variant);
    client.stop();

    std::cout << "PROGRAM COMPLETE\n";
    return 0;
}
