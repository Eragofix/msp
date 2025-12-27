#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <iomanip>
#include <algorithm>

#include "udp_comm_module.hpp"
#include "integral_state_feedback.hpp"

std::atomic<bool> run_program{true};
void signalHandler(int) { run_program = false; }

/* HIL main program */

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    const int local_port = (argc > 1) ? std::stoi(argv[1]) : 4002;
    const std::string remote_host = (argc > 2) ? std::string(argv[2]) : "192.168.0.26";
    const int remote_port = (argc > 3) ? std::stoi(argv[3]) : 4005;

    const double phi0 = (argc > 4) ? std::stod(argv[4]) : -0.5;    // rad
    const double wn = (argc > 5) ? std::stod(argv[5]) : 10.0;      // rad/s
    const double zeta_d = (argc > 6) ? std::stod(argv[6]) : 0.7;  // [-]
    const double pI = (argc > 7) ? std::stod(argv[7]) : 1.0;       // integrator pole (default)

    UDPCommModule comm(local_port, remote_host, remote_port);

    ModelParameters params;

    const double omega_phi0 = 0.0;

    IntegralStateFeedbackController ic(params, phi0, omega_phi0, wn, zeta_d, pI);

    std::cout << "HIL integral state-feedback started\n";
    std::cout << std::fixed << std::setprecision(4);

    long long counter = 0;
    auto last_time = std::chrono::steady_clock::now();
    const double max_dt = 0.001;

    while (run_program) {
        double phi = 0.0, omega = 0.0;
        if (!comm.receive(phi, omega)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_time;
        last_time = now;
        double dt = elapsed.count();
        if (dt <= 0.0) dt = 1e-6;
        if (dt > max_dt) dt = max_dt;

        double u = ic.compute(phi, omega, dt);
        // saturate output to +/-400
        double u_sat = std::clamp(u, -400.0, 400.0);
        comm.send(u_sat);

        if ((counter++ % 50) == 0) {
            std::cout << "phi=" << phi << " | omega=" << omega
                      << " | u=" << u
                      << " | u0=" << ic.getU0()
                      << " | k1=" << ic.getK1()
                      << " | k2=" << ic.getK2()
                      << " | Fi=" << ic.getFi()
                      << " | xi=" << ic.getXi()
                      << "\r" << std::flush;
        }
    }

    std::cout << "\nHIL stopped.\n";
    return 0;
}