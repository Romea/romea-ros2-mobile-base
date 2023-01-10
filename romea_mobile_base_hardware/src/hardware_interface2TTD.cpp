// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>
#include <vector>

// local
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2TTD::HardwareInterface2TTD(
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
    hardware_info.joints[RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID]),
  front_left_roller_wheel_spinning_joint_feedback_(
    hardware_info.joints[FRONT_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID]),
  front_right_roller_wheel_spinning_joint_feedback_(
    hardware_info.joints[FRONT_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID]),
  rear_left_roller_wheel_spinning_joint_feedback_(
    hardware_info.joints[REAR_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID]),
  rear_right_roller_wheel_spinning_joint_feedback_(
    hardware_info.joints[REAR_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID])
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2TTD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  left_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  right_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  front_left_roller_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  front_right_roller_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_left_roller_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_right_roller_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2TTD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
HardwareCommand2TD HardwareInterface2TTD::get_command()const
{
  return {left_sprocket_wheel_spinning_joint_.get_command(),
      right_sprocket_wheel_spinning_joint_.get_command()};
}

//-----------------------------------------------------------------------------
void HardwareInterface2TTD::set_state(const HardwareState2TD & hardware_state)
{
  left_sprocket_wheel_spinning_joint_.
  set_state(hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.
  set_state(hardware_state.rightSprocketWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void HardwareInterface2TTD::set_state(
  const HardwareState2TD & hardware_state,
  const RotationalMotionState & left_idler_wheel_spinning_set_point,
  const RotationalMotionState & right_idler_wheel_spinning_set_point,
  const RotationalMotionState & front_left_roller_wheel_spinning_set_point,
  const RotationalMotionState & front_right_roller_wheel_spinning_set_point,
  const RotationalMotionState & rear_left_roller_wheel_spinning_set_point,
  const RotationalMotionState & rear_right_roller_wheel_spinning_set_point)
{
  set_state(hardware_state);

  left_idler_wheel_spinning_joint_feedback_.
  set_state(left_idler_wheel_spinning_set_point);
  right_idler_wheel_spinning_joint_feedback_.
  set_state(right_idler_wheel_spinning_set_point);

  front_left_roller_wheel_spinning_joint_feedback_.
  set_state(front_left_roller_wheel_spinning_set_point);
  front_right_roller_wheel_spinning_joint_feedback_.
  set_state(front_right_roller_wheel_spinning_set_point);
  rear_left_roller_wheel_spinning_joint_feedback_.
  set_state(rear_left_roller_wheel_spinning_set_point);
  rear_right_roller_wheel_spinning_joint_feedback_.
  set_state(rear_right_roller_wheel_spinning_set_point);
}

}  // namespace romea
