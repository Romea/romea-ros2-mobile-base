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

#ifndef ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__HARDWARE_INFO_COMMON_HPP_
#define ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__HARDWARE_INFO_COMMON_HPP_

// std
#include <string>

// ros
#include <hardware_interface/hardware_info.hpp>
#include <romea_core_common/lexical/LexicalCast.hpp>

namespace romea::ros2
{

bool has_parameter(
  const hardware_interface::HardwareInfo & hardware_info, const std::string & parameter_name);

const std::string & get_parameter(
  const hardware_interface::HardwareInfo & hardware_info, const std::string & parameter_name);

template<typename T>
T get_parameter(
  const hardware_interface::HardwareInfo & hardware_info, const std::string & parameter_name)
{
  std::string parameter = get_parameter(hardware_info, parameter_name);
  return romea::core::lexical_cast<T>(parameter);
}

template<typename T>
T get_parameter_or(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & parameter_name,
  const T & default_value)
{
  try {
    return get_parameter<T>(hardware_info, parameter_name);
  } catch (const std::runtime_error &) {
    return default_value;
  }
}

const hardware_interface::ComponentInfo & get_joint_info(
  const hardware_interface::HardwareInfo & hardware_info, const std::string & joint_name);

double get_wheelbase(const hardware_interface::HardwareInfo & hardware_info);

double get_front_track(const hardware_interface::HardwareInfo & hardware_info);

double get_front_wheel_radius(const hardware_interface::HardwareInfo & hardware_info);

double get_front_hub_carrier_offset(const hardware_interface::HardwareInfo & hardware_info);

double get_rear_track(const hardware_interface::HardwareInfo & hardware_info);

double get_rear_wheel_radius(const hardware_interface::HardwareInfo & hardware_info);

double get_rear_hub_carrier_offset(const hardware_interface::HardwareInfo & hardware_info);

double get_idler_wheel_radius(const hardware_interface::HardwareInfo & hardware_info);

double get_roller_wheel_radius(const hardware_interface::HardwareInfo & hardware_info);

double get_sprocket_wheel_radius(const hardware_interface::HardwareInfo & hardware_info);

double get_track_thickness(const hardware_interface::HardwareInfo & hardware_info);

}  // namespace romea::ros2

#endif  // ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__HARDWARE_INFO_COMMON_HPP_
