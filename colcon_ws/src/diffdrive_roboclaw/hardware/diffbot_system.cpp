// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_roboclaw/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_roboclaw
{
  hardware_interface::CallbackReturn DiffDriveRoboclawHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    config_.left_front_wheel_joint = info_.hardware_parameters["left_front_wheel_joint"];
    config_.left_back_wheel_joint = info_.hardware_parameters["left_back_wheel_joint"];
    config_.right_front_wheel_joint = info_.hardware_parameters["right_front_wheel_joint"];
    config_.right_back_wheel_joint = info_.hardware_parameters["right_back_wheel_joint"];
    config_.device = info_.hardware_parameters["device"];
    config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    config_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    config_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    // init wheels
    wheel_lf_.setup(config_.left_front_wheel_joint, config_.enc_counts_per_rev);
    wheel_lb_.setup(config_.left_back_wheel_joint, config_.enc_counts_per_rev);
    wheel_rf_.setup(config_.right_front_wheel_joint, config_.enc_counts_per_rev);
    wheel_rb_.setup(config_.right_back_wheel_joint, config_.enc_counts_per_rev);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveRoboclawHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveRoboclawHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveRoboclawHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveRoboclawHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveRoboclawHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveRoboclawHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // left front wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_lf_.name, hardware_interface::HW_IF_POSITION, &wheel_lf_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_lf_.name, hardware_interface::HW_IF_VELOCITY, &wheel_lf_.vel));

    // left back wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_lb_.name, hardware_interface::HW_IF_POSITION, &wheel_lb_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_lb_.name, hardware_interface::HW_IF_VELOCITY, &wheel_lb_.vel));

    // right front wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_rf_.name, hardware_interface::HW_IF_POSITION, &wheel_rf_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_rf_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rf_.vel));

    // right back wheel
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_rb_.name, hardware_interface::HW_IF_POSITION, &wheel_rb_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_rb_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rb_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveRoboclawHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // left front wheel
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_lf_.name, hardware_interface::HW_IF_VELOCITY, &wheel_lf_.cmd));

    // left back wheel
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_lb_.name, hardware_interface::HW_IF_VELOCITY, &wheel_lb_.cmd));

    // right front wheel
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_rf_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rf_.cmd));

    // right back wheel
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_rb_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rb_.cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveRoboclawHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboclawHardware"), "Activating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboclawHardware"), config_.device.c_str());
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboclawHardware"), std::to_string(config_.baud_rate).c_str());
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboclawHardware"), std::to_string(config_.timeout_ms).c_str());

    roboclaw_comms_.connect(config_.device, config_.baud_rate, config_.timeout_ms);

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboclawHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveRoboclawHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboclawHardware"), "Deactivating ...please wait...");

    roboclaw_comms_.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboclawHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffDriveRoboclawHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    roboclaw_comms_.read_encoder_values(wheel_lf_.enc, wheel_lb_.enc, wheel_rf_.enc, wheel_rb_.enc);

    double delta_seconds = period.seconds();

    double pos_prev = wheel_lf_.pos;
    wheel_lf_.pos = wheel_lf_.calc_enc_angle();
    wheel_lf_.vel = (wheel_lf_.pos - pos_prev) / delta_seconds;

    pos_prev = wheel_lb_.pos;
    wheel_lb_.pos = wheel_lb_.calc_enc_angle();
    wheel_lb_.vel = (wheel_lb_.pos - pos_prev) / delta_seconds;

    pos_prev = wheel_rf_.pos;
    wheel_rf_.pos = wheel_rf_.calc_enc_angle();
    wheel_rf_.vel = (wheel_rf_.pos - pos_prev) / delta_seconds;

    pos_prev = wheel_rb_.pos;
    wheel_rb_.pos = wheel_rb_.calc_enc_angle();
    wheel_rb_.vel = (wheel_rb_.pos - pos_prev) / delta_seconds;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type diffdrive_roboclaw ::DiffDriveRoboclawHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboclawHardware"), "Writing...");

    int motor_lf_qpps = wheel_lf_.cmd / wheel_lf_.rads_per_count;
    int motor_lb_qpps = wheel_lb_.cmd / wheel_lb_.rads_per_count;
    int motor_rf_qpps = wheel_rf_.cmd / wheel_rf_.rads_per_count;
    int motor_rb_qpps = wheel_rb_.cmd / wheel_rb_.rads_per_count;
    roboclaw_comms_.set_motor_values(motor_lf_qpps, motor_lb_qpps, motor_rf_qpps, motor_rb_qpps);

    return hardware_interface::return_type::OK;
  }

} // namespace diffdrive_roboclaw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diffdrive_roboclaw::DiffDriveRoboclawHardware, hardware_interface::SystemInterface)
