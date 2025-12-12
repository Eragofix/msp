// MotorSender.hpp - component that constructs SetMotor message and sends it
#ifndef MOTOR_SENDER_HPP
#define MOTOR_SENDER_HPP

#include <msp_msg.hpp>
#include <Client.hpp>
#include <array>

class MotorSender {
public:
    MotorSender(msp::client::Client& client, msp::FirmwareVariant fw_variant)
        : client_(client), fw_variant_(fw_variant) {}

    // send four motors (motor5..8 will be zeroed)
    int send(const std::array<uint16_t,4>& motors) {
        msp::msg::SetMotor msg(fw_variant_);
        for (size_t i = 0; i < 4; ++i) msg.motor[i] = motors[i];
        for (size_t i = 4; i < 8; ++i) msg.motor[i] = 0;

        // store last message for logging
        for (size_t i = 0; i < 4; ++i) last_msg_[i] = motors[i];

        last_result_ = client_.sendMessage(msg);
        return last_result_;
    }

    std::array<uint16_t,4> getLastMsg() const { return last_msg_; }
    int getLastResult() const { return last_result_; }

private:
    msp::client::Client& client_;
    msp::FirmwareVariant fw_variant_;
    std::array<uint16_t,4> last_msg_ = {0,0,0,0};
    int last_result_ = 0;
};

#endif // MOTOR_SENDER_HPP
