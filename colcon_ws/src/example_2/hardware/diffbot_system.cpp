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

#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_2
{
  hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
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
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
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

  std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
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

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

    roboclaw_comms_.connect(config_.device, config_.baud_rate, config_.timeout_ms);

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

    roboclaw_comms_.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffBotSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {

    //  roboclaw_comms_.read_encoder_values();

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ros2_control_demo_example_2 ::DiffBotSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");

    //  roboclaw_comms_.set_motor_values();

    return hardware_interface::return_type::OK;
  }

} // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)