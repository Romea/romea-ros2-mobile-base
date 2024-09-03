// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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


// std
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_utils/ros2_control/info/hardware_info4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface4WD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HardwareInterface4WD::HardwareInterface4WD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: front_left_wheel_spinning_joint_(
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo4WD::get_front_left_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  front_right_wheel_spinning_joint_(
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo4WD::get_front_right_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  rear_left_wheel_spinning_joint_(
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo4WD::get_rear_left_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  rear_right_wheel_spinning_joint_(
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo4WD::get_rear_right_wheel_spinning_joint_info(hardware_info),
    command_interface_type)
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> HardwareInterface4WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface> HardwareInterface4WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
core::HardwareCommand4WD HardwareInterface4WD::get_hardware_command() const
{
  // *INDENT-OFF*
  return {front_left_wheel_spinning_joint_.get_command(),
      front_right_wheel_spinning_joint_.get_command(),
      rear_left_wheel_spinning_joint_.get_command(),
      rear_right_wheel_spinning_joint_.get_command()};
  // *INDENT-ON*
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState HardwareInterface4WD::get_joint_state_command() const
{
  auto joint_states = make_joint_state_msg(4);
  front_left_wheel_spinning_joint_.write_command(joint_states);
  front_right_wheel_spinning_joint_.write_command(joint_states);
  rear_left_wheel_spinning_joint_.write_command(joint_states);
  rear_right_wheel_spinning_joint_.write_command(joint_states);
  return joint_states;
}

//-----------------------------------------------------------------------------
void HardwareInterface4WD::set_feedback(const core::HardwareState4WD & hardware_state)
{
  front_left_wheel_spinning_joint_.set_feedback(hardware_state.frontLeftWheelSpinningMotion);
  front_right_wheel_spinning_joint_.set_feedback(hardware_state.frontRightWheelSpinningMotion);
  rear_left_wheel_spinning_joint_.set_feedback(hardware_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.set_feedback(hardware_state.rearRightWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void HardwareInterface4WD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  front_left_wheel_spinning_joint_.read_feedback(joint_states);
  front_right_wheel_spinning_joint_.read_feedback(joint_states);
  rear_left_wheel_spinning_joint_.read_feedback(joint_states);
  rear_right_wheel_spinning_joint_.read_feedback(joint_states);
}


}  // namespace ros2
}  // namespace romea
