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
#include "romea_mobile_base_hardware/hardware_interface2TD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HardwareInterface2TD::HardwareInterface2TD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_sprocket_wheel_spinning_joint_(
    hardware_info.joints[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_sprocket_wheel_spinning_joint_(
    hardware_info.joints[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  left_idler_wheel_spinning_joint_feedback_(
    hardware_info.joints[LEFT_IDLER_WHEEL_SPINNING_JOINT_ID]),
  right_idler_wheel_spinning_joint_feedback_(
    hardware_info.joints[RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID])
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2TD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  left_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  right_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2TD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
core::HardwareCommand2TD HardwareInterface2TD::get_command()const
{
  // *INDENT-OFF*
  return {left_sprocket_wheel_spinning_joint_.get_command(),
      right_sprocket_wheel_spinning_joint_.get_command()};
  // *INDENT-ON*
}


//-----------------------------------------------------------------------------
void HardwareInterface2TD::set_state(const core::HardwareState2TD & hardware_state)
{
  left_sprocket_wheel_spinning_joint_.
  set_state(hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.
  set_state(hardware_state.rightSprocketWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void HardwareInterface2TD::set_state(
  const core::HardwareState2TD & hardware_state,
  const core::RotationalMotionState & left_idler_wheel_spinning_motion,
  const core::RotationalMotionState & right_idler_wheel_spinning_motion)
{
  set_state(hardware_state);

  left_idler_wheel_spinning_joint_feedback_.
  set_state(left_idler_wheel_spinning_motion);
  right_idler_wheel_spinning_joint_feedback_.
  set_state(right_idler_wheel_spinning_motion);
}

}  // namespace ros2
}  // namespace romea
