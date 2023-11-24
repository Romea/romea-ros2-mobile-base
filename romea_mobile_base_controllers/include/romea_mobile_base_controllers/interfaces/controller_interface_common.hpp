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


#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE_COMMON_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE_COMMON_HPP_

// std
#include <string>


// ros
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// romea ros
#include "romea_common_utils/ros_versions.hpp"

namespace romea
{
namespace ros2
{

#if ROS_DISTRO == ROS_GALACTIC
using HardwareInterfaceNode = rclcpp::Node;
#else
using HardwareInterfaceNode = rclcpp_lifecycle::LifecycleNode;
#endif

std::string hardware_position_interface_name(const std::string joint_name);

std::string hardware_velocity_interface_name(const std::string joint_name);

std::string hardware_effort_interface_name(const std::string joint_name);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE_COMMON_HPP_
