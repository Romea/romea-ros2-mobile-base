// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>
#include <vector>

// local
#include "romea_mobile_base_hardware/hardware_interface2TD.hpp"

namespace romea
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
HardwareCommand2TD HardwareInterface2TD::get_command()const
{
  return {left_sprocket_wheel_spinning_joint_.get_command(),
      right_sprocket_wheel_spinning_joint_.get_command()};
}


//-----------------------------------------------------------------------------
void HardwareInterface2TD::set_state(const HardwareState2TD & hardware_state)
{
  left_sprocket_wheel_spinning_joint_.
  set_state(hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.
  set_state(hardware_state.rightSprocketWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void HardwareInterface2TD::set_state(
  const HardwareState2TD & hardware_state,
  const RotationalMotionState & left_idler_wheel_spinning_motion,
  const RotationalMotionState & right_idler_wheel_spinning_motion)
{
  set_state(hardware_state);

  left_idler_wheel_spinning_joint_feedback_.
  set_state(left_idler_wheel_spinning_motion);
  right_idler_wheel_spinning_joint_feedback_.
  set_state(right_idler_wheel_spinning_motion);
}

}  // namespace romea
