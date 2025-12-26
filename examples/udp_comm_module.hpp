#pragma once

#include <asio.hpp>
#include <vector>
#include <string>

class UDPCommModule {
public:
    UDPCommModule(int local_port, const std::string& remote_host, int remote_port);
    bool receive(double& out_phi, double& out_omega);
    void send(double control_signal);

private:
    asio::io_context io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint remote_endpoint_;
};
