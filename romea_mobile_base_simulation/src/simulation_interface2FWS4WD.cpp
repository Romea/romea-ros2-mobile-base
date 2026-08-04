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

// romea
#include "romea_mobile_base_simulation/simulation_interface2FWS4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface_base.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const HardwareInterface2FWS4WD::Configuration & configuration)
{
  return {
    make_gazebo_joint_info(
      configuration.front_left_wheel_steering_joint_info, hardware_interface::HW_IF_POSITION),
    make_gazebo_joint_info(
      configuration.front_right_wheel_steering_joint_info, hardware_interface::HW_IF_POSITION),
    make_gazebo_joint_info(
      configuration.front_left_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.front_right_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.rear_left_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.rear_right_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type)};
}

}  // namespace ros2
}  // namespace romea
