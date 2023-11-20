#ifndef DIFFDRIVE_ROBOCLAW_ROBOCLAW_COMMS
#define DIFFDRIVE_ROBOCLAW_ROBOCLAW_COMMS

#include <sstream>
#include <iostream>

#include "roboclaw_serial_driver/roboclaw.h"

class RoboclawComms
{

public:
    RoboclawComms() = default;

    void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        rc = roboclaw_init(serial_device.c_str(), baud_rate);
    }

    void disconnect()
    {
        roboclaw_close(rc);
    }

    // bool connected() const
    // {
    //     return roboclaw_connections->isConnected();
    // }

    void read_encoder_values(int &val_1, int &val_2, int &val_3, int &val_4)
    {
        roboclaw_encoders(rc, left_addr, &val_1, &val_2);
        roboclaw_encoders(rc, right_addr, &val_3, &val_4);
    }

    void set_motor_values(int motor_lf_qpps, int motor_lb_qpps, int motor_rf_qpps, int motor_rb_qpps)
    {
        roboclaw_speed_m1m2(rc, left_addr, motor_lf_qpps, motor_lb_qpps);
        roboclaw_speed_m1m2(rc, right_addr, motor_rf_qpps, motor_rb_qpps);
    }

private:
    struct roboclaw *rc;
    int timeout_ms_;
    uint8_t left_addr = 0x81;
    uint8_t right_addr = 0x80;
};

#endif // DIFFDRIVE_ROBOCLAW_ROBOCLAW_COMMS