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
#include <map>
#include <string>

// romea
#include "romea_mobile_base_hardware/hardware_interface2WD.hpp"

// local
#include "romea_mobile_base_gazebo/gazebo_interface2WD.hpp"


namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
GazeboInterface2WD::GazeboInterface2WD(
  gz::sim::EntityComponentManager & ecm,
  std::map<std::string, gz::sim::Entity> & enable_joints,
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_wheel_spinning_joint_(ecm, enable_joints,
    hardware_info.joints[HardwareInterface2WD::LEFT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_wheel_spinning_joint_(ecm, enable_joints,
    hardware_info.joints[HardwareInterface2WD::RIGHT_WHEEL_SPINNING_JOINT_ID],
    command_interface_type)
{
}


//-----------------------------------------------------------------------------
core::SimulationState2WD GazeboInterface2WD::get_state() const
{
  return {left_wheel_spinning_joint_.get_state(),
      right_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface2WD::set_command(const core::SimulationCommand2WD & command)
{
  left_wheel_spinning_joint_.set_command(command.leftWheelSpinningSetPoint);
  right_wheel_spinning_joint_.set_command(command.rightWheelSpinningSetPoint);
}

}  // namespace ros2
}  // namespace romea
