#include "AngularRateSensor.hpp"
#include <cmath>
#include <iostream>

AngularRateSensor::AngularRateSensor(msp::client::Client& client, msp::FirmwareVariant fw_variant)
    : client_(client), raw_imu_msg_(fw_variant),
      omega_roll_rad_(0.0), omega_pitch_rad_(0.0), omega_yaw_rad_(0.0) {}

bool AngularRateSensor::update() {
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

double AngularRateSensor::getOmegaRollRad() const { return omega_roll_rad_; }
