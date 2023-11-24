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
#include "romea_mobile_base_hardware/hardware_interface2THD.hpp"


// local
#include "romea_mobile_base_gazebo/gazebo_interface2THD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
GazeboInterface2THD::GazeboInterface2THD(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_sprocket_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2THD::LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_sprocket_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2THD::RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  front_left_idler_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2THD::FRONT_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  front_right_idler_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2THD::FRONT_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_left_idler_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2THD::REAR_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_right_idler_wheel_spinning_joint_(parent_model,
    hardware_info.joints[HardwareInterface2THD::REAR_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type)
{
}

//-----------------------------------------------------------------------------
core::SimulationState2THD GazeboInterface2THD::get_state() const
{
  return {left_sprocket_wheel_spinning_joint_.get_state(),
    right_sprocket_wheel_spinning_joint_.get_state(),
    front_left_idler_wheel_spinning_joint_.get_state(),
    front_right_idler_wheel_spinning_joint_.get_state(),
    rear_left_idler_wheel_spinning_joint_.get_state(),
    rear_right_idler_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface2THD::set_command(const core::SimulationCommand2THD & command)
{
  left_sprocket_wheel_spinning_joint_.set_command(command.leftSprocketWheelSpinningSetPoint);
  right_sprocket_wheel_spinning_joint_.set_command(command.rightSprocketWheelSpinningSetPoint);
  front_left_idler_wheel_spinning_joint_.set_command(command.frontLeftIdlerWheelSpinningSetPoint);
  front_right_idler_wheel_spinning_joint_.set_command(command.frontRightIdlerWheelSpinningSetPoint);
  rear_left_idler_wheel_spinning_joint_.set_command(command.rearLeftIdlerWheelSpinningSetPoint);
  rear_right_idler_wheel_spinning_joint_.set_command(command.rearRightIdlerWheelSpinningSetPoint);
}

////-----------------------------------------------------------------------------
// void write(const HardwareInterface2THD & hardware_interface,
//           const double & left_idler_wheels_speed_command,
//           const double & right_idler_wheels_speed_command,
//           GazeboInterface2THD & gazebo_interface)
//{

//  write(hardware_interface.left_sprocket_wheel_spinning_joint,
//        gazebo_interface.left_sprocket_wheel_spinning_joint);
//  write(hardware_interface.right_sprocket_wheel_spinning_joint,
//        gazebo_interface.right_sprocket_wheel_spinning_joint);

//  gazebo_interface.front_left_idler_wheel_spinning_joint.
//      setCommand(left_idler_wheels_speed_command);
//  gazebo_interface.rear_left_idler_wheel_spinning_joint.
//      setCommand(left_idler_wheels_speed_command);

//  gazebo_interface.front_right_idler_wheel_spinning_joint.
//      setCommand(right_idler_wheels_speed_command);
//  gazebo_interface.rear_right_idler_wheel_spinning_joint.
//      setCommand(right_idler_wheels_speed_command);

//}

////-----------------------------------------------------------------------------
// void write(const HardwareInterface2THD & hardware_interface,
//           GazeboInterface2THD & gazebo_interface)
//{

//  const double ratio = gazebo_interface.idler_wheel_radius/
//      gazebo_interface.sprocket_wheel_radius;

//  double left_idler_wheels_speed_command = ratio*
//      hardware_interface.left_sprocket_wheel_spinning_joint.command.get();
//  double right_idler_wheels_speed_command = ratio*
//      hardware_interface.right_sprocket_wheel_spinning_joint.command.get();

//  write(hardware_interface,
//        left_idler_wheels_speed_command,
//        right_idler_wheels_speed_command,
//        gazebo_interface);

//}

////-----------------------------------------------------------------------------
// void read(const GazeboInterface2THD & gazebo_interface,
//          const SpinningJointGazeboInterface::Feedback & left_sprocket_wheel_feedback,
//          const SpinningJointGazeboInterface::Feedback & right_sprocket_wheel_feedback,
//          HardwareInterface2THD & hardware_interface)
//{

//  read(left_sprocket_wheel_feedback,
//       hardware_interface.left_sprocket_wheel_spinning_joint.feedback);
//  read(right_sprocket_wheel_feedback,
//       hardware_interface.right_sprocket_wheel_spinning_joint.feedback);

//  read(gazebo_interface.front_left_idler_wheel_spinning_joint,
//       hardware_interface.front_left_idler_wheel_spinning_joint_feedback);
//  read(gazebo_interface.front_right_idler_wheel_spinning_joint,
//       hardware_interface.front_right_idler_wheel_spinning_joint_feedback);

//  read(gazebo_interface.rear_left_idler_wheel_spinning_joint,
//       hardware_interface.rear_left_idler_wheel_spinning_joint_feedback);
//  read(gazebo_interface.rear_right_idler_wheel_spinning_joint,
//       hardware_interface.rear_right_idler_wheel_spinning_joint_feedback);

//}


////-----------------------------------------------------------------------------
// void read(const GazeboInterface2THD & gazebo_interface,
//          HardwareInterface2THD & hardware_interface)
//{

//  auto left_sprocket_wheel_feedback=drive_wheel_feedback(
//        gazebo_interface.left_sprocket_wheel_spinning_joint,
//        gazebo_interface.front_left_idler_wheel_spinning_joint,
//        gazebo_interface.rear_left_idler_wheel_spinning_joint,
//        gazebo_interface.sprocket_wheel_radius,
//        gazebo_interface.idler_wheel_radius);

//  auto right_sprocket_wheel_feedback=drive_wheel_feedback(
//        gazebo_interface.right_sprocket_wheel_spinning_joint,
//        gazebo_interface.front_right_idler_wheel_spinning_joint,
//        gazebo_interface.rear_right_idler_wheel_spinning_joint,
//        gazebo_interface.sprocket_wheel_radius,
//        gazebo_interface.idler_wheel_radius);

//  read(gazebo_interface,
//       left_sprocket_wheel_feedback,
//       right_sprocket_wheel_feedback,
//       hardware_interface);

//}

}  // namespace ros2
}  // namespace romea
