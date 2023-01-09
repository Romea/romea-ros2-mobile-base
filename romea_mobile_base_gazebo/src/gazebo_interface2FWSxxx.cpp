// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_mobile_base_hardware/hardware_interface2FWS2FWD.hpp>

// std
#include <string>

// local
#include "romea_mobile_base_gazebo/gazebo_interface2FWSxxx.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2FWSxxx::GazeboInterface2FWSxxx(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: front_left_wheel_steering_joint_(parent_model,
    hardware_info.joints[HardwareInterface2FWS2FWD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint_(parent_model,
    hardware_info.joints[HardwareInterface2FWS2FWD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2FWS2FWD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  front_right_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2FWS2FWD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_left_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2FWS2FWD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_right_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2FWS2FWD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type)
{
}

//-----------------------------------------------------------------------------
SimulationState2FWSxxx GazeboInterface2FWSxxx::get_state() const
{
  return {front_left_wheel_steering_joint_.get_state(),
      front_right_wheel_steering_joint_.get_state(),
      front_left_wheel_spinning_joint_.get_state(),
      front_right_wheel_spinning_joint_.get_state(),
      rear_left_wheel_spinning_joint_.get_state(),
      rear_right_wheel_spinning_joint_.get_state()};
}


//-----------------------------------------------------------------------------
void GazeboInterface2FWSxxx::set_command(const SimulationCommand2FWSxxx & command)
{
  front_left_wheel_steering_joint_.set_command(command.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_command(command.frontRightWheelSteeringAngle);
  front_left_wheel_spinning_joint_.set_command(command.frontLeftWheelSpinningSetPoint);
  front_right_wheel_spinning_joint_.set_command(command.frontRightWheelSpinningSetPoint);
  rear_left_wheel_spinning_joint_.set_command(command.rearLeftWheelSpinningSetPoint);
  rear_right_wheel_spinning_joint_.set_command(command.rearRightWheelSpinningSetPoint);
}

}  // namespace romea
