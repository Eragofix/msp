#include "AttitudeSensor.hpp"
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

AttitudeSensor::AttitudeSensor(msp::client::Client& client, msp::FirmwareVariant fw_variant)
    : client_(client), attitude_msg_(fw_variant), roll_rad_(0.0) {}

bool AttitudeSensor::update() {
    if (client_.sendMessage(attitude_msg_) != 1) {
        std::cerr << "[AttitudeSensor] Failed to read attitude\n";
        return false;
    }
    double roll_deg = static_cast<double>(attitude_msg_.roll);
    roll_rad_ = roll_deg * M_PI / 180.0;
    return true;
}

double AttitudeSensor::getRollRad() const { return roll_rad_; }
