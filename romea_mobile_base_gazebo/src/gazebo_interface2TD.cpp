// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_mobile_base_hardware/hardware_interface2TD.hpp>

// std
#include <string>

// local
#include "romea_mobile_base_gazebo/gazebo_interface2TD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2TD::GazeboInterface2TD(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_sprocket_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TD::LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_sprocket_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TD::RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  left_idler_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TD::LEFT_IDLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_idler_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TD::RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type)
{
}

//-----------------------------------------------------------------------------
SimulationState2TD GazeboInterface2TD::get_state() const
{
  return {left_sprocket_wheel_spinning_joint_.get_state(),
      right_sprocket_wheel_spinning_joint_.get_state(),
      left_idler_wheel_spinning_joint_.get_state(),
      right_idler_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface2TD::set_command(const SimulationCommand2TD & command)
{
  left_sprocket_wheel_spinning_joint_.set_command(command.leftSprocketWheelSpinningSetPoint);
  right_sprocket_wheel_spinning_joint_.set_command(command.rightSprocketWheelSpinningSetPoint);
  left_idler_wheel_spinning_joint_.set_command(command.leftIdlerWheelSpinningSetPoint);
  right_idler_wheel_spinning_joint_.set_command(command.rightIdlerWheelSpinningSetPoint);
}

////-----------------------------------------------------------------------------
// void write(const HardwareInterface2TD & hardware_interface,
//           GazeboInterface2TD & gazebo_interface)
//{
//  write(hardware_interface.left_sprocket_wheel_spinning_joint,
//        gazebo_interface.left_sprocket_wheel_spinning_joint);
//  write(hardware_interface.right_sprocket_wheel_spinning_joint,
//        gazebo_interface.right_sprocket_wheel_spinning_joint);

//  gazebo_interface.left_idler_wheel_spinning_joint.
//      setCommand(hardware_interface.left_sprocket_wheel_spinning_joint.command.get());
//  gazebo_interface.right_idler_wheel_spinning_joint.
//      setCommand(hardware_interface.right_sprocket_wheel_spinning_joint.command.get());

//}

////-----------------------------------------------------------------------------
// void read(const GazeboInterface2TD & gazebo_interface,
//          const SpinningJointGazeboInterface::Feedback & left_sprocket_wheel_feedback,
//          const SpinningJointGazeboInterface::Feedback & right_sprocket_wheel_feedback,
//          HardwareInterface2TD & hardware_interface)
//{
//  read(left_sprocket_wheel_feedback,
//       hardware_interface.left_sprocket_wheel_spinning_joint.feedback);
//  read(right_sprocket_wheel_feedback,
//       hardware_interface.right_sprocket_wheel_spinning_joint.feedback);

//  read(gazebo_interface.left_idler_wheel_spinning_joint,
//       hardware_interface.left_idler_wheel_spinning_joint_feedback);
//  read(gazebo_interface.right_idler_wheel_spinning_joint,
//       hardware_interface.right_idler_wheel_spinning_joint_feedback);

//}

////-----------------------------------------------------------------------------
// void read(const GazeboInterface2TD & gazebo_interface,
//          HardwareInterface2TD & hardware_interface)
//{

//  auto left_sprocket_wheel_feedback=drive_wheel_feedback(
//        gazebo_interface.left_sprocket_wheel_spinning_joint,
//        gazebo_interface.left_idler_wheel_spinning_joint);

//  auto right_sprocket_wheel_feedback=drive_wheel_feedback(
//        gazebo_interface.right_sprocket_wheel_spinning_joint,
//        gazebo_interface.right_idler_wheel_spinning_joint);

//  read(gazebo_interface,
//       left_sprocket_wheel_feedback,
//       right_sprocket_wheel_feedback,
//       hardware_interface);
//}

}  // namespace romea
