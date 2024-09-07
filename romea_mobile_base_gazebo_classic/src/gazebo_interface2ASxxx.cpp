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
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2ASxxx.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS4WD.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2ASxxx.hpp"

namespace romea
{
namespace ros2
{

// front_axle_steering_joint_(parent_model,
//     HardwareInfo2ASxxx::get_front_axle_steering_joint_info(hardware_info)),
//   rear_axle_steering_joint_(parent_model,
//     HardwareInfo2ASxxx::get_rear_axle_steering_joint_info(hardware_info)),
//   front_left_wheel_steering_joint_(parent_model,
//     HardwareInfo2ASxxx::get_front_left_wheel_steering_joint_info(hardware_info)),
//   front_right_wheel_steering_joint_(parent_model,
//     HardwareInfo2ASxxx::get_front_right_wheel_steering_joint_info(hardware_info)),
//   rear_left_wheel_steering_joint_(parent_model,
//     HardwareInfo2ASxxx::get_front_left_wheel_steering_joint_info(hardware_info)),
//   rear_right_wheel_steering_joint_(parent_model,
//     HardwareInfo2ASxxx::get_rear_right_wheel_steering_joint_info(hardware_info)),
//   front_left_wheel_spinning_joint_(parent_model,
//     HardwareInfo2ASxxx::get_front_left_wheel_spinning_joint_info(hardware_info),
//     command_interface_type),
//   front_right_wheel_spinning_joint_(parent_model,
//     HardwareInfo2ASxxx::get_front_right_wheel_spinning_joint_info(hardware_info),
//     command_interface_type),
//   rear_left_wheel_spinning_joint_(parent_model,
//     HardwareInfo2ASxxx::get_rear_left_wheel_spinning_joint_info(hardware_info),
//     command_interface_type),
//   rear_right_wheel_spinning_joint_(parent_model,
//     HardwareInfo2ASxxx::get_rear_right_wheel_spinning_joint_info(hardware_info),
//     command_interface_type)
//-----------------------------------------------------------------------------
GazeboInterface2ASxxx::GazeboInterface2ASxxx(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: front_axle_steering_joint_(parent_model,
    HardwareInfo2ASxxx::get_front_axle_steering_joint_info(hardware_info)),
  rear_axle_steering_joint_(parent_model,
    HardwareInfo2ASxxx::get_rear_axle_steering_joint_info(hardware_info)),
  front_left_wheel_steering_joint_(parent_model,
    HardwareInfo2ASxxx::get_front_left_wheel_steering_joint_info(hardware_info)),
  front_right_wheel_steering_joint_(parent_model,
    HardwareInfo2ASxxx::get_front_right_wheel_steering_joint_info(hardware_info)),
  rear_left_wheel_steering_joint_(parent_model,
    HardwareInfo2ASxxx::get_rear_left_wheel_steering_joint_info(hardware_info)),
  rear_right_wheel_steering_joint_(parent_model,
    HardwareInfo2ASxxx::get_rear_right_wheel_steering_joint_info(hardware_info)),
  front_left_wheel_spinning_joint_(parent_model,
    HardwareInfo2ASxxx::get_front_left_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  front_right_wheel_spinning_joint_(parent_model,
    HardwareInfo2ASxxx::get_front_right_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  rear_left_wheel_spinning_joint_(parent_model,
    HardwareInfo2ASxxx::get_rear_left_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  rear_right_wheel_spinning_joint_(parent_model,
    HardwareInfo2ASxxx::get_rear_right_wheel_spinning_joint_info(hardware_info),
    command_interface_type)
{
}

//-----------------------------------------------------------------------------
core::SimulationState2ASxxx GazeboInterface2ASxxx::get_state() const
{
  return{front_axle_steering_joint_.get_state(),
      front_left_wheel_steering_joint_.get_state(),
      front_right_wheel_steering_joint_.get_state(),
      rear_axle_steering_joint_.get_state(),
      rear_left_wheel_steering_joint_.get_state(),
      rear_right_wheel_steering_joint_.get_state(),
      front_left_wheel_spinning_joint_.get_state(),
      front_right_wheel_spinning_joint_.get_state(),
      rear_left_wheel_spinning_joint_.get_state(),
      rear_right_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface2ASxxx::set_command(const core::SimulationCommand2ASxxx & command)
{
  front_axle_steering_joint_.set_command(command.frontAxleSteeringAngle);
  rear_axle_steering_joint_.set_command(command.rearAxleSteeringAngle);

  front_left_wheel_steering_joint_.set_command(command.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_command(command.frontRightWheelSteeringAngle);
  rear_left_wheel_steering_joint_.set_command(command.rearLeftWheelSteeringAngle);
  rear_right_wheel_steering_joint_.set_command(command.rearRightWheelSteeringAngle);
  front_left_wheel_spinning_joint_.set_command(command.frontLeftWheelSpinningSetPoint);
  front_right_wheel_spinning_joint_.set_command(command.frontRightWheelSpinningSetPoint);
  rear_left_wheel_spinning_joint_.set_command(command.rearLeftWheelSpinningSetPoint);
  rear_right_wheel_spinning_joint_.set_command(command.rearRightWheelSpinningSetPoint);
}

}  // namespace ros2
}  // namespace romea
