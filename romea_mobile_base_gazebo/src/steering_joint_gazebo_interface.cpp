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

// ros
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// local
#include "romea_mobile_base_gazebo/steering_joint_gazebo_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SteeringJointGazeboInterface::SteeringJointGazeboInterface(
  gz::sim::EntityComponentManager & ecm,
  std::map<std::string, gz::sim::Entity> & enable_joints,
  const hardware_interface::ComponentInfo & joint_info)
: JointGazeboInterface(ecm, enable_joints, joint_info), position_(0.0)
{
  create_state_gazebo_component_<gz::sim::components::JointPosition>();
  create_command_gazebo_component_<gz::sim::components::JointVelocityCmd>();
}

//-----------------------------------------------------------------------------
void SteeringJointGazeboInterface::set_command(const double & command)
{
  double velocity_command = 100.0 * (command - position_);
  //  std::cout << " command " << command <<" position "<< position_
  //   <<" velocity_command " << velocity_command*180/3.14 << std::endl;
  set_command_<gz::sim::components::JointVelocityCmd>(velocity_command);
  // previous_command_ = command;
}

//-----------------------------------------------------------------------------
double SteeringJointGazeboInterface::get_state() const
{
  position_ = get_state_<gz::sim::components::JointPosition>();
  return position_;
}

}  // namespace ros2
}  // namespace romea
