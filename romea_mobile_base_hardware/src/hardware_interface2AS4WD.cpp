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
#include "romea_mobile_base_hardware/hardware_interface2AS4WD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HardwareInterface2AS4WD::HardwareInterface2AS4WD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: front_axle_steering_joint_(
    hardware_info.joints[FRONT_AXLE_STEERING_JOINT_ID]),
  rear_axle_steering_joint_(
    hardware_info.joints[REAR_AXLE_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint_(
    hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  front_right_wheel_spinning_joint_(
    hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  rear_left_wheel_spinning_joint_(
    hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint_(
    hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  front_left_wheel_steering_joint_feedback_(
    hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID],
    hardware_interface::HW_IF_POSITION),
  front_right_wheel_steering_joint_feedback_(
    hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID],
    hardware_interface::HW_IF_POSITION),
  rear_left_wheel_steering_joint_feedback_(
    hardware_info.joints[REAR_LEFT_WHEEL_STEERING_JOINT_ID],
    hardware_interface::HW_IF_POSITION),
  rear_right_wheel_steering_joint_feedback_(
    hardware_info.joints[REAR_RIGHT_WHEEL_STEERING_JOINT_ID],
    hardware_interface::HW_IF_POSITION)
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2AS4WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_axle_steering_joint_.export_state_interface(state_interfaces);
  rear_axle_steering_joint_.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_left_wheel_steering_joint_feedback_.export_interface(state_interfaces);
  front_right_wheel_steering_joint_feedback_.export_interface(state_interfaces);
  rear_left_wheel_steering_joint_feedback_.export_interface(state_interfaces);
  rear_right_wheel_steering_joint_feedback_.export_interface(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2AS4WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_axle_steering_joint_.export_command_interface(command_interfaces);
  rear_axle_steering_joint_.export_command_interface(command_interfaces);
  front_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
core::HardwareCommand2AS4WD HardwareInterface2AS4WD::get_command()const
{
  // *INDENT-OFF*
  return {front_axle_steering_joint_.get_command(),
      rear_axle_steering_joint_.get_command(),
      front_left_wheel_spinning_joint_.get_command(),
      front_right_wheel_spinning_joint_.get_command(),
      rear_left_wheel_spinning_joint_.get_command(),
      rear_right_wheel_spinning_joint_.get_command()};
  // *INDENT-ON*
}

//-----------------------------------------------------------------------------
void HardwareInterface2AS4WD::set_state(const core::HardwareState2AS4WD & hardware_state)
{
  front_axle_steering_joint_.set_state(hardware_state.frontAxleSteeringAngle);
  rear_axle_steering_joint_.set_state(hardware_state.rearAxleSteeringAngle);

  front_left_wheel_spinning_joint_.set_state(hardware_state.frontLeftWheelSpinningMotion);
  front_right_wheel_spinning_joint_.set_state(hardware_state.frontRightWheelSpinningMotion);
  rear_left_wheel_spinning_joint_.set_state(hardware_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.set_state(hardware_state.rearRightWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void HardwareInterface2AS4WD::set_state(
  const core::HardwareState2AS4WD & hardware_state,
  const core::SteeringAngleState & front_left_wheel_steering_angle,
  const core::SteeringAngleState & front_right_wheel_steering_angle,
  const core::SteeringAngleState & rear_left_wheel_steering_angle,
  const core::SteeringAngleState & rear_right_wheel_steering_angle)
{
  set_state(hardware_state);

  front_left_wheel_steering_joint_feedback_.set(front_left_wheel_steering_angle);
  front_right_wheel_steering_joint_feedback_.set(front_right_wheel_steering_angle);
  rear_left_wheel_steering_joint_feedback_.set(rear_left_wheel_steering_angle);
  rear_right_wheel_steering_joint_feedback_.set(rear_right_wheel_steering_angle);
}

}  // namespace ros2
}  // namespace romea
