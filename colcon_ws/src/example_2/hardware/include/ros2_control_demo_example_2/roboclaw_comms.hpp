#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <iostream>
#include "libroboclaw/include/libroboclaw/roboclaw_driver.h"
#include "libroboclaw/libroboclawConfig.h"

class RoboclawComms
{

public:
    RoboclawComms() = default;

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        // roboclaw_connections = new libroboclaw::driver(serial_device, baud_rate);
        roboclaw_connections = new libroboclaw::driver("/dev/ttyACM0", 38400);
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
    }

    void set_motor_values(int motor_lf_qpps, int motor_lb_qpps, int motor_rf_qpps, int motor_rb_qpps)
    {
        roboclaw_connections->set_velocity(left_addr, {motor_lf_qpps, motor_lb_qpps});
        roboclaw_connections->set_velocity(right_addr, {motor_rf_qpps, motor_rb_qpps});
    }

private:
    libroboclaw::driver *roboclaw_connections;
    int timeout_ms_;
    unsigned int left_addr = 128;
    unsigned int right_addr = 129;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP