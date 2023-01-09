// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_mobile_base_hardware/hardware_interface2WD.hpp>

// std
#include <string>

// local
#include "romea_mobile_base_gazebo/gazebo_interface2WD.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2WD::GazeboInterface2WD(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2WD::LEFT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2WD::RIGHT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type)
{
}


//-----------------------------------------------------------------------------
SimulationState2WD GazeboInterface2WD::get_state() const
{
  return {left_wheel_spinning_joint_.get_state(),
      right_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface2WD::set_command(const SimulationCommand2WD & command)
{
  left_wheel_spinning_joint_.set_command(command.leftWheelSpinningSetPoint);
  right_wheel_spinning_joint_.set_command(command.rightWheelSpinningSetPoint);
}

}  // namespace romea
