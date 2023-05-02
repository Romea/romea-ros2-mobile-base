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
#include "romea_mobile_base_hardware/hardware_interface2FWS2FWD.hpp"

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
