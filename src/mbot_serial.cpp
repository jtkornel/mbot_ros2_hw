// Copyright (c) 2023, Jan Tore Korneliussen
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <limits>
#include <vector>

#include <boards.hpp>
#include <comm.hpp>

#include "mbot_hardware/mbot_serial.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mbot_hardware
{
hardware_interface::CallbackReturn MbotSerial::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check number and type of command interfaces
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp ::get_logger("MbotSerialHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp ::get_logger("MbotSerialHardware"),
        "Command interface '%s' of joint '%s' is not '%s'.",
        joint.command_interfaces[0].name.c_str(), joint.name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
    }

    // Check number and type of state interfaces, should have position and velocity (in that order)

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp ::get_logger("MbotSerialHardware"),
        "Joint '%s' has %zu state interfaces found. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp ::get_logger("MbotSerialHardware"),
        "State interface '%s' of joint '%s' is not '%s'.", joint.state_interfaces[0].name.c_str(),
        joint.name.c_str(), hardware_interface::HW_IF_POSITION);
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp ::get_logger("MbotSerialHardware"),
        "State interface '%s' of joint '%s' is not '%s'.", joint.state_interfaces[1].name.c_str(),
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
    }
  }

  io_context_ = std::make_shared<asio::io_context>();
  mbot_com_ =
    std::make_shared<libmbot::Comm>(*io_context_, info_.hardware_parameters["serial_port"]);

  mbot_board_ = std::make_shared<libmbot::AurigaBoard>(*mbot_com_);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MbotSerial::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MbotSerial::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MbotSerial::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MbotSerial::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MbotSerial::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MbotSerial::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read position from motor 1 (in degrees)
  auto maybe_pos0 = mbot_board_->m_motor_1.get_pos.request();

  if (maybe_pos0)
  {
    double pos0_rad = (*maybe_pos0) * 2.0 * M_PI / 360.0;
    hw_position_states_[0] = pos0_rad;
  }

  // Read position from motor 2 (in degrees)
  auto maybe_pos1 = mbot_board_->m_motor_2.get_pos.request();

  if (maybe_pos1)
  {
    double pos1_rad = (*maybe_pos1) * 2.0 * M_PI / 360.0;
    hw_position_states_[1] = pos1_rad;
  }

  // Read velocity from motor 1 (in RPM)
  auto maybe_vel0 = mbot_board_->m_motor_1.get_speed.request();

  if (maybe_vel0)
  {
    double vel0_rad_s = (*maybe_vel0) / 60.0f * 2.0 * M_PI;

    hw_velocity_states_[0] = vel0_rad_s;
  }

  // Read velocity from motor 2 (in RPM)
  auto maybe_vel1 = mbot_board_->m_motor_2.get_speed.request();

  if (maybe_vel1)
  {
    double vel1_rad_s = (*maybe_vel1) / 60.0f * 2.0 * M_PI;

    hw_velocity_states_[1] = vel1_rad_s;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MbotSerial::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  float vel0_rpm = hw_velocity_commands_[0] / (2.0 * M_PI) * 60.0;
  mbot_board_->m_motor_1.set_speed_motion.request(vel0_rpm);

  float vel1_rpm = hw_velocity_commands_[1] / (2.0 * M_PI) * 60.0;
  mbot_board_->m_motor_2.set_speed_motion.request(vel1_rpm);

  return hardware_interface::return_type::OK;
}

}  // namespace mbot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mbot_hardware::MbotSerial, hardware_interface::SystemInterface)
