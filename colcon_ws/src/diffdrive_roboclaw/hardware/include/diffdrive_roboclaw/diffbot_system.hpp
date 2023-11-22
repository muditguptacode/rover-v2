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

#ifndef DIFFDRIVE_ROBOCLAW__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_ROBOCLAW__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffdrive_roboclaw/visibility_control.h"

#include "diffdrive_roboclaw/roboclaw_comms.hpp"
#include "diffdrive_roboclaw/wheel.hpp"

namespace diffdrive_roboclaw
{
  class DiffDriveRoboclawHardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      std::string left_front_wheel_joint = "";
      std::string left_back_wheel_joint = "";
      std::string right_front_wheel_joint = "";
      std::string right_back_wheel_joint = "";
      std::string device = "";
      int baud_rate = 0;
      int timeout_ms = 0;
      int enc_counts_per_rev = 0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveRoboclawHardware)

    DIFFDRIVE_ROBOCLAW_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    DIFFDRIVE_ROBOCLAW_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DIFFDRIVE_ROBOCLAW_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DIFFDRIVE_ROBOCLAW_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROBOCLAW_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROBOCLAW_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROBOCLAW_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    DIFFDRIVE_ROBOCLAW_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    DIFFDRIVE_ROBOCLAW_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    RoboclawComms roboclaw_comms_;
    Config config_;
    Wheel wheel_lf_;
    Wheel wheel_lb_;
    Wheel wheel_rf_;
    Wheel wheel_rb_;
    double main_battery_voltage_;
  };

} // namespace diffdrive_roboclaw

#endif // DIFFDRIVE_ROBOCLAW__DIFFBOT_SYSTEM_HPP_
