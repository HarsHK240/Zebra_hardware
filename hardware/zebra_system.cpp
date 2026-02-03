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

#include "include/zebra/zebra_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <cstring>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace zebra
{

hardware_interface::CallbackReturn ZebraSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Load Parameters from URDF ---
  try {
    // cfg_.left_wheel_name = info_.hardware_parameters.at("left_wheel_name");
    // cfg_.right_wheel_name = info_.hardware_parameters.at("right_wheel_name");
    cfg_.loop_rate = std::stof(info_.hardware_parameters.at("loop_rate"));
    cfg_.device = info_.hardware_parameters.at("device");
    cfg_.baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters.at("timeout_ms"));
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters.at("enc_counts_per_rev"));
  } catch (const std::out_of_range &e) {
    RCLCPP_FATAL(rclcpp::get_logger("ZebraSystemHardware"), "Missing a required hardware parameter in URDF! %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Resize State/Command Vectors ---
  // We need 1 slot per joint for Position, Velocity, and Command
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // --- Validate Interfaces ---
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("ZebraSystemHardware"), "Joint '%s' has incorrect command interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2 || 
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("ZebraSystemHardware"), "Joint '%s' has incorrect state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZebraSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ZebraSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ZebraSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ZebraSystemHardware"), "Activating... Opening Serial Port %s", cfg_.device.c_str());
  
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  if (!comms_.connected()) {
    RCLCPP_FATAL(rclcpp::get_logger("ZebraSystemHardware"), "Failed to connect to Arduino!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset values
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0;
    hw_velocities_[i] = 0;
    hw_commands_[i] = 0;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ZebraSystemHardware"), "Successfully Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZebraSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ZebraSystemHardware"), "Deactivating... Closing Serial Port");
  comms_.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("ZebraSystemHardware"), "Successfully Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZebraSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  double left_ticks = 0.0;
  double right_ticks = 0.0;

  // 1. Get raw ticks from Arduino
  if(comms_.read_encoder_values(left_ticks, right_ticks))
{
  // 2. Convert Ticks -> Radians
  // Formula: (ticks / ticks_per_rev) * 2 * PI
  double left_rads = (left_ticks / cfg_.enc_counts_per_rev) * 2.0 * M_PI;
  double right_rads = (right_ticks / cfg_.enc_counts_per_rev) * 2.0 * M_PI;

  // 3. Assign to Joints
  // Assuming URDF order: left_back, right_back, right_front, left_front
  // Map left encoder to ALL left wheels, right encoder to ALL right wheels
  if (hw_positions_.size() > 0) hw_positions_[0] = left_rads;
  if (hw_positions_.size() > 3) hw_positions_[3] = left_rads;

  if (hw_positions_.size() > 1) hw_positions_[1] = right_rads;
  if (hw_positions_.size() > 2) hw_positions_[2] = right_rads;

  // Note: We are not calculating velocity here. 
  // The diff_drive_controller will calculate velocity from position changes automatically.
}
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZebraSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Get angular velocity commands (rad/s) from ROS
  double left_vel_rads = (hw_commands_.size() > 0) ? hw_commands_[0] : 0.0;
  double right_vel_rads = (hw_commands_.size() > 1) ? hw_commands_[1] : 0.0;

  // Convert Angular (rad/s) -> Linear (m/s) for Arduino
  // Use a hardcoded radius or fetch from config if you added it
  const double WHEEL_RADIUS = 0.05; 
  double left_mps = left_vel_rads * WHEEL_RADIUS;
  double right_mps = right_vel_rads * WHEEL_RADIUS;

  // Send to Arduino
  comms_.set_motor_values(left_mps, right_mps);

  return hardware_interface::return_type::OK;
}

}  // namespace zebra

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  zebra::ZebraSystemHardware, hardware_interface::SystemInterface)