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

// romea
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"


// local
#include "romea_mobile_base_gazebo/gazebo_interface2TTD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2TTD::GazeboInterface2TTD(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_sprocket_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TTD::LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_sprocket_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TTD::RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  left_idler_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TTD::LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_idler_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TTD::RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  front_left_roller_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TTD::FRONT_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  front_right_roller_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TTD::FRONT_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_left_roller_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TTD::REAR_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_right_roller_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2TTD::REAR_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type)
{
}

//-----------------------------------------------------------------------------
SimulationState2TTD GazeboInterface2TTD::get_state() const
{
  return {left_sprocket_wheel_spinning_joint_.get_state(),
      right_sprocket_wheel_spinning_joint_.get_state(),
      left_idler_wheel_spinning_joint_.get_state(),
      right_idler_wheel_spinning_joint_.get_state(),
      front_left_roller_wheel_spinning_joint_.get_state(),
      front_right_roller_wheel_spinning_joint_.get_state(),
      rear_left_roller_wheel_spinning_joint_.get_state(),
      rear_right_roller_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface2TTD::set_command(const SimulationCommand2TTD & command)
{
  left_sprocket_wheel_spinning_joint_.set_command(
    command.leftSprocketWheelSpinningSetPoint);
  right_sprocket_wheel_spinning_joint_.set_command(
    command.rightSprocketWheelSpinningSetPoint);
  left_idler_wheel_spinning_joint_.set_command(
    command.leftIdlerWheelSpinningSetPoint);
  right_idler_wheel_spinning_joint_.set_command(
    command.rightIdlerWheelSpinningSetPoint);
  front_left_roller_wheel_spinning_joint_.set_command(
    command.frontLeftRollerWheelSpinningSetPoint);
  front_right_roller_wheel_spinning_joint_.set_command(
    command.frontRightRollerWheelSpinningSetPoint);
  rear_left_roller_wheel_spinning_joint_.set_command(
    command.rearLeftRollerWheelSpinningSetPoint);
  rear_right_roller_wheel_spinning_joint_.set_command(
    command.rearRightRollerWheelSpinningSetPoint);
}

////-----------------------------------------------------------------------------
// void write(const HardwareInterface2TTD & hardware_interface,
//           const double & left_roller_wheels_speed_command,
//           const double & right_roller_wheels_speed_command,
//           GazeboInterface2TTD & gazebo_interface)
//{

//  write(hardware_interface.left_sprocket_wheel_spinning_joint,
//        gazebo_interface.left_sprocket_wheel_spinning_joint);
//  write(hardware_interface.right_sprocket_wheel_spinning_joint,
//        gazebo_interface.right_sprocket_wheel_spinning_joint);

////  gazebo_interface.front_left_idler_wheel_spinning_joint.
////      setCommand(left_idler_wheels_speed_command);
////  gazebo_interface.rear_left_idler_wheel_spinning_joint.
////      setCommand(left_idler_wheels_speed_command);

////  gazebo_interface.front_right_idler_wheel_spinning_joint.
////      setCommand(right_idler_wheels_speed_command);
////  gazebo_interface.rear_right_idler_wheel_spinning_joint.
////      setCommand(right_idler_wheels_speed_command);

//}

////-----------------------------------------------------------------------------
// void write(const HardwareInterface2TTD & hardware_interface,
//           GazeboInterface2TTD & gazebo_interface)
//{

//  const double ratio = gazebo_interface.roller_wheel_radius/
//      gazebo_interface.sprocket_wheel_radius;

////  double left_idler_wheels_speed_command = ratio*
////      hardware_interface.left_sprocket_wheel_spinning_joint.command.get();
////  double right_idler_wheels_speed_command = ratio*
////      hardware_interface.right_sprocket_wheel_spinning_joint.command.get();

////  write(hardware_interface,
////        left_idler_wheels_speed_command,
////        right_idler_wheels_speed_command,
////        gazebo_interface);

//}

////-----------------------------------------------------------------------------
// void read(const GazeboInterface2TTD & gazebo_interface,
//          const SpinningJointGazeboInterface::Feedback & left_sprocket_wheel_feedback,
//          const SpinningJointGazeboInterface::Feedback & right_sprocket_wheel_feedback,
//          HardwareInterface2TTD & hardware_interface)
//{

////  read(left_sprocket_wheel_feedback,
////       hardware_interface.left_sprocket_wheel_spinning_joint.feedback);
////  read(right_sprocket_wheel_feedback,
////       hardware_interface.right_sprocket_wheel_spinning_joint.feedback);

////  read(gazebo_interface.front_left_idler_wheel_spinning_joint,
////       hardware_interface.front_left_idler_wheel_spinning_joint_feedback);
////  read(gazebo_interface.front_right_idler_wheel_spinning_joint,
////       hardware_interface.front_right_idler_wheel_spinning_joint_feedback);

////  read(gazebo_interface.rear_left_idler_wheel_spinning_joint,
////       hardware_interface.rear_left_idler_wheel_spinning_joint_feedback);
////  read(gazebo_interface.rear_right_idler_wheel_spinning_joint,
////       hardware_interface.rear_right_idler_wheel_spinning_joint_feedback);

//}


////-----------------------------------------------------------------------------
// void read(const GazeboInterface2TTD & gazebo_interface,
//          HardwareInterface2TTD & hardware_interface)
//{

////  auto left_sprocket_wheel_feedback=drive_wheel_feedback(
////        gazebo_interface.left_sprocket_wheel_spinning_joint,
////        gazebo_interface.front_left_idler_wheel_spinning_joint,
////        gazebo_interface.rear_left_idler_wheel_spinning_joint,
////        gazebo_interface.sprocket_wheel_radius,
////        gazebo_interface.idler_wheel_radius);

////  auto right_sprocket_wheel_feedback=drive_wheel_feedback(
////        gazebo_interface.right_sprocket_wheel_spinning_joint,
////        gazebo_interface.front_right_idler_wheel_spinning_joint,
////        gazebo_interface.rear_right_idler_wheel_spinning_joint,
////        gazebo_interface.sprocket_wheel_radius,
////        gazebo_interface.idler_wheel_radius);

////  read(gazebo_interface,
////       left_sprocket_wheel_feedback,
////       right_sprocket_wheel_feedback,
////       hardware_interface);

//}

}  // namespace romea
