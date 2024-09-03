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


#ifndef ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__JOINT_INFO_HPP_
#define ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__JOINT_INFO_HPP_

// std
#include <string>

// ros
#include "hardware_interface/hardware_info.hpp"


namespace romea
{
namespace ros2
{

const hardware_interface::InterfaceInfo &
get_command_interface_info(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & interface_name);

const hardware_interface::InterfaceInfo &
get_state_interface_info(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & interface_name);


}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__JOINT_INFO_HPP_
