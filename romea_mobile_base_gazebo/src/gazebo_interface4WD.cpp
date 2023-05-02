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
