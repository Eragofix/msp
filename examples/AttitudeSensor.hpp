#pragma once

#include <Client.hpp>
#include <msp_msg.hpp>

class AttitudeSensor {
public:
    AttitudeSensor(msp::client::Client& client, msp::FirmwareVariant fw_variant);
    bool update();
    double getRollRad() const;

private:
    msp::client::Client& client_;
    msp::msg::Attitude attitude_msg_;
    double roll_rad_;
};
