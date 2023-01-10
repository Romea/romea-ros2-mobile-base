// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>
#include <vector>

// local
#include "romea_mobile_base_hardware/hardware_interface2FWS4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2FWS4WD::HardwareInterface2FWS4WD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: front_left_wheel_steering_joint_(
    hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint_(
    hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
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
    spinning_joint_command_interface_type)
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2FWS4WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_left_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_right_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2FWS4WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_left_wheel_steering_joint_.export_command_interface(command_interfaces);
  front_right_wheel_steering_joint_.export_command_interface(command_interfaces);
  front_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
HardwareCommand2FWS4WD HardwareInterface2FWS4WD::get_command()const
{
  return {front_left_wheel_steering_joint_.get_command(),
      front_right_wheel_steering_joint_.get_command(),
      front_left_wheel_spinning_joint_.get_command(),
      front_right_wheel_spinning_joint_.get_command(),
      rear_left_wheel_spinning_joint_.get_command(),
      rear_right_wheel_spinning_joint_.get_command()};
}

//-----------------------------------------------------------------------------
void HardwareInterface2FWS4WD::set_state(const HardwareState2FWS4WD & hardware_state)
{
  front_left_wheel_steering_joint_.
  set_state(hardware_state.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.
  set_state(hardware_state.frontRightWheelSteeringAngle);

  front_left_wheel_spinning_joint_.
  set_state(hardware_state.frontLeftWheelSpinningMotion);
  front_right_wheel_spinning_joint_.
  set_state(hardware_state.frontRightWheelSpinningMotion);

  rear_left_wheel_spinning_joint_.
  set_state(hardware_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.
  set_state(hardware_state.rearRightWheelSpinningMotion);
}

}  // namespace romea
