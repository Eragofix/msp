#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <iomanip>

#include "udp_comm_module.hpp"
#include "state_feedback.hpp"

std::atomic<bool> run_program{true};
void signalHandler(int) { run_program = false; }

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    const int local_port = (argc > 1) ? std::stoi(argv[1]) : 4002;
    const std::string remote_host = (argc > 2) ? std::string(argv[2]) : "192.168.0.26";
    const int remote_port = (argc > 3) ? std::stoi(argv[3]) : 4005;

    // Optional: phi0, wn, zeta_d as CLI args
    const double phi0 = (argc > 4) ? std::stod(argv[4]) : -0.5;    // rad
    const double wn = (argc > 5) ? std::stod(argv[5]) : 10.0;      // rad/s
    const double zeta_d = (argc > 6) ? std::stod(argv[6]) : 0.7;  // [-]

    UDPCommModule comm(local_port, remote_host, remote_port);

    // model parameters (defaults in header)
    ModelParameters params;

    // state-feedback setpoints / dynamics
    const double omega_phi0 = 0.0;

    // instantiate controller
    StateFeedbackController sf(params, phi0, omega_phi0, wn, zeta_d);

    std::cout << "HIL state-feedback started\n";
    std::cout << std::fixed << std::setprecision(4);

    long long counter = 0;
    while (run_program) {
        double phi = 0.0, omega = 0.0;
        if (!comm.receive(phi, omega)) {
            // no packet received, sleep briefly
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        double u = sf.compute(phi, omega);
        comm.send(u);

        if ((counter++ % 50) == 0) {
            std::cout << "phi=" << phi << " | omega=" << omega
                      << " | u=" << u
                      << " | u0=" << sf.getU0()
                      << " | k1=" << sf.getK1()
                      << " | k2=" << sf.getK2()
                      << "\r" << std::flush;
        }
    }

    std::cout << "\nHIL stopped.\n";
    return 0;
}
