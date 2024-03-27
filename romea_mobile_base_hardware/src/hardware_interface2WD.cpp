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

// local
#include "romea_mobile_base_hardware/hardware_interface2WD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HardwareInterface2WD::HardwareInterface2WD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_wheel_spinning_joint_(
    LEFT_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[LEFT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_wheel_spinning_joint_(
    RIGHT_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[RIGHT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type)
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}


//-----------------------------------------------------------------------------
core::HardwareCommand2WD HardwareInterface2WD::get_command()const
{
  // *INDENT-OFF*
  return {left_wheel_spinning_joint_.get_command(),
      right_wheel_spinning_joint_.get_command()};
  // *INDENT-ON*
}

//-----------------------------------------------------------------------------
core::HardwareCommand2WD HardwareInterface2WD::get_hardware_command() const
{
  return get_command();
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState HardwareInterface2WD::get_joint_state_command() const
{
  auto joint_states = make_joint_state_msg(2);
  left_wheel_spinning_joint_.write_command(joint_states);
  right_wheel_spinning_joint_.write_command(joint_states);
  return joint_states;
}


//-----------------------------------------------------------------------------
void HardwareInterface2WD::set_state(const core::HardwareState2WD & hardware_state)
{
  left_wheel_spinning_joint_.set_state(hardware_state.leftWheelSpinningMotion);
  right_wheel_spinning_joint_.set_state(hardware_state.rightWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void HardwareInterface2WD::set_feedback(const core::HardwareState2WD & hardware_state)
{
  set_state(hardware_state);
}

//-----------------------------------------------------------------------------
void HardwareInterface2WD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  left_wheel_spinning_joint_.read_feedback(joint_states);
  right_wheel_spinning_joint_.read_feedback(joint_states);
}

}  // namespace ros2
}  // namespace romea
