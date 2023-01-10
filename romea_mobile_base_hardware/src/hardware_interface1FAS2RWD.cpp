// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>
#include <vector>

// local
#include "romea_mobile_base_hardware/hardware_interface1FAS2RWD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface1FAS2RWD::HardwareInterface1FAS2RWD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: front_axle_steering_joint_(hardware_info.joints[FRONT_AXLE_STEERING_JOINT_ID]),
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
  front_left_wheel_spinning_joint_feedback_(
    hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]),
  front_right_wheel_spinning_joint_feedback_(
    hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID])
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface1FAS2RWD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_axle_steering_joint_.export_state_interface(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_left_wheel_steering_joint_feedback_.export_interface(state_interfaces);
  front_right_wheel_steering_joint_feedback_.export_interface(state_interfaces);
  front_left_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface1FAS2RWD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_axle_steering_joint_.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
HardwareCommand1FAS2RWD HardwareInterface1FAS2RWD::get_command()const
{
  return {front_axle_steering_joint_.get_command(),
      rear_left_wheel_spinning_joint_.get_command(),
      rear_right_wheel_spinning_joint_.get_command()};
}

//-----------------------------------------------------------------------------
void HardwareInterface1FAS2RWD::set_state(const HardwareState1FAS2RWD & hardware_state)
{
  front_axle_steering_joint_.set_state(hardware_state.frontAxleSteeringAngle);
  rear_left_wheel_spinning_joint_.set_state(hardware_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.set_state(hardware_state.rearRightWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void HardwareInterface1FAS2RWD::set_state(
  const HardwareState1FAS2RWD & hardware_state,
  const SteeringAngleState & front_left_wheel_steering_angle,
  const SteeringAngleState & front_right_wheel_steering_angle,
  const RotationalMotionState & front_left_wheel_spin_motion,
  const RotationalMotionState & front_right_wheel_spin_motion)
{
  set_state(hardware_state);

  front_left_wheel_steering_joint_feedback_.set(front_left_wheel_steering_angle);
  front_right_wheel_steering_joint_feedback_.set(front_right_wheel_steering_angle);

  front_left_wheel_spinning_joint_feedback_.set_state(front_left_wheel_spin_motion);
  front_right_wheel_spinning_joint_feedback_.set_state(front_right_wheel_spin_motion);
}

}  // namespace romea
