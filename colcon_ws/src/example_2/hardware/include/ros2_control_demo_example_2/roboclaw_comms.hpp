#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <iostream>
#include "libroboclaw/include/libroboclaw/roboclaw_driver.h"

class RoboclawComms
{

public:
    RoboclawComms() = default;

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        roboclaw_connections = new libroboclaw::driver(serial_device, baud_rate);
    }

    void disconnect()
    {
        roboclaw_connections->close();
    }

    bool connected() const
    {
        return roboclaw_connections->isConnected();
    }

    void read_encoder_values(int &val_1, int &val_2, int &val_3, int &val_4)
    {
        std::pair<int, int> enc_res_left = roboclaw_connections->get_encoders(left_addr);
        std::pair<int, int> enc_res_right = roboclaw_connections->get_encoders(right_addr);
        val_1 = enc_res_left.first;
        val_2 = enc_res_left.second;
        val_3 = enc_res_right.first;
        val_4 = enc_res_right.second;
        // val_1 = std::atoi(token_1.c_str());
        // val_2 = std::atoi(token_2.c_str());
    }
    void set_motor_values(int val_1, int val_2)
    {
    }

private:
    libroboclaw::driver *roboclaw_connections;
    int timeout_ms_;
    unsigned int left_addr = 128;
    unsigned int right_addr = 129;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP