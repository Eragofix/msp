#pragma once

#include <Client.hpp>
#include <msp_msg.hpp>

class AngularRateSensor {
public:
    AngularRateSensor(msp::client::Client& client, msp::FirmwareVariant fw_variant);
    bool update();
    double getOmegaRollRad() const;

private:
    msp::client::Client& client_;
    msp::msg::RawImu raw_imu_msg_;
    double omega_roll_rad_;
    double omega_pitch_rad_;
    double omega_yaw_rad_;
};
