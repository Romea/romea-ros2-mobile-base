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


// string
#include <map>
#include <string>

// ros
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// local
#include "romea_mobile_base_gazebo/spinning_joint_gazebo_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SpinningJointGazeboInterface::SpinningJointGazeboInterface(
  gz::sim::EntityComponentManager & ecm,
  std::map<std::string, gz::sim::Entity> & enable_joints,
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & command_interface_type):
  JointGazeboInterface(ecm, enable_joints, joint_info)
{
  if (!command_interface_type.compare(hardware_interface::HW_IF_VELOCITY)) {
    control_type = core::RotationalMotionControlType::VELOCITY;
  } else if (!command_interface_type.compare(hardware_interface::HW_IF_EFFORT)) {
    control_type = core::RotationalMotionControlType::TORQUE;
  } else {
    // throw error
  }

  create_state_gazebo_component_<gz::sim::components::JointPosition>();
  create_state_gazebo_component_<gz::sim::components::JointVelocity>();
  create_state_gazebo_component_<gz::sim::components::JointTransmittedWrench>();
  create_command_gazebo_component_<gz::sim::components::JointVelocityCmd>();
}

//-----------------------------------------------------------------------------
void SpinningJointGazeboInterface::set_command(const double & command)
{
  if (control_type == core::RotationalMotionControlType::VELOCITY) {
    set_command_<gz::sim::components::JointVelocityCmd>(command);
  } else {
    set_command_<gz::sim::components::JointForceCmd>(command);
  }
}

//-----------------------------------------------------------------------------
core::RotationalMotionState SpinningJointGazeboInterface::get_state() const
{
  core::RotationalMotionState state;
  state.position = get_state_<gz::sim::components::JointPosition>();
  state.velocity = get_state_<gz::sim::components::JointVelocity>();
  state.torque = get_state_<gz::sim::components::JointTransmittedWrench>();
  return state;
}

}  // namespace ros2
}  // namespace romea
