// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// romea
#include "romea_mobile_base_hardware/hardware_interface4WD.hpp"


// local
#include "romea_mobile_base_gazebo/gazebo_interface4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface4WD::GazeboInterface4WD(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: front_left_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface4WD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  front_right_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface4WD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_left_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface4WD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_right_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface4WD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type)
{
}

//-----------------------------------------------------------------------------
SimulationState4WD GazeboInterface4WD::get_state() const
{
  return {front_left_wheel_spinning_joint_.get_state(),
      front_right_wheel_spinning_joint_.get_state(),
      rear_left_wheel_spinning_joint_.get_state(),
      rear_right_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface4WD::set_command(const SimulationCommand4WD & command)
{
  front_left_wheel_spinning_joint_.set_command(command.frontLeftWheelSpinningSetPoint);
  front_right_wheel_spinning_joint_.set_command(command.frontRightWheelSpinningSetPoint);
  rear_left_wheel_spinning_joint_.set_command(command.rearLeftWheelSpinningSetPoint);
  rear_right_wheel_spinning_joint_.set_command(command.rearRightWheelSpinningSetPoint);
}

}  // namespace romea
