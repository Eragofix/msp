#include "udp_comm_module.hpp"
#include <asio.hpp>
#include <iostream>

using asio::ip::udp;

UDPCommModule::UDPCommModule(int local_port, const std::string& remote_host, int remote_port)
    : socket_(io_context_, udp::endpoint(udp::v4(), local_port)),
      remote_endpoint_(asio::ip::make_address(remote_host), remote_port)
{
    asio::socket_base::receive_buffer_size option(1024 * 64);
    socket_.set_option(option);
}

bool UDPCommModule::receive(double& out_phi, double& out_omega) {
    std::vector<double> buffer(2); // [phi, omega]
    udp::endpoint sender_endpoint;
    asio::error_code ec;

    size_t len = socket_.receive_from(asio::buffer(buffer), sender_endpoint, 0, ec);
    if (!ec && len == 2 * sizeof(double)) {
        out_phi = buffer[0];
        out_omega = buffer[1];
        return true;
    }
    return false;
}

void UDPCommModule::send(double control_signal) {
    asio::error_code ec;
    socket_.send_to(asio::buffer(&control_signal, sizeof(double)), remote_endpoint_, 0, ec);
}
