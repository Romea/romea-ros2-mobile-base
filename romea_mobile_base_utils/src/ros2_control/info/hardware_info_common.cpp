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
#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

// romea
#include "romea_core_common/lexical/LexicalCast.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"


namespace
{

// //-----------------------------------------------------------------------------
// bool has_parameter(
//   const hardware_interface::HardwareInfo & hardware_info,
//   const std::string & parameter_name)
// {
//   return hardware_info.hardware_parameters.find(parameter_name) !=
//          hardware_info.hardware_parameters.end();
// }

//-----------------------------------------------------------------------------
const std::string & get_parameter(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & parameter_name)
{
  auto it = hardware_info.hardware_parameters.find(parameter_name);
  if (it == hardware_info.hardware_parameters.end()) {
    std::stringstream msg;
    msg << " Unable to get hardware parameter ";
    msg << parameter_name;
    msg << ", check your urdf description file";
    throw(std::runtime_error(msg.str()));
  } else {
    return it->second;
  }
}


//-----------------------------------------------------------------------------
template<typename T>
T get_parameter(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & parameter_name)
{
  std::string parameter = get_parameter(hardware_info, parameter_name);

  if constexpr (std::is_same_v<T, std::string>) {
    return parameter;
  } else {
    return romea::core::lexical_cast<T>(parameter);
  }
}

// //-----------------------------------------------------------------------------
// template<typename T>
// T get_parameter_or(
//   const hardware_interface::HardwareInfo & hardware_info,
//   const std::string & parameter_name,
//   const T & default_value)
// {
//   if (has_parameter(hardware_info, parameter_name)) {
//     return get_parameter<T>(hardware_info, parameter_name);
//   } else {
//     return default_value;
//   }
// }

}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
const hardware_interface::ComponentInfo &
get_joint_info(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & joint_name)
{
  const auto & joint = std::find_if(
    hardware_info.joints.begin(),
    hardware_info.joints.end(),
    [&joint_name](const auto & joint)
    {
      return joint.name == joint_name &&
      joint.type == "joint";
    });

  if (joint == hardware_info.joints.cend()) {
    std::stringstream ss;
    ss << " Unable to obtain info of joint ";
    ss << joint_name;
    throw(std::runtime_error(ss.str()));
  }

  return *joint;
}


//-----------------------------------------------------------------------------
double get_wheelbase(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "wheelbase");
}

//-----------------------------------------------------------------------------
double get_front_track(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "front_track");
}

//-----------------------------------------------------------------------------
double get_front_wheel_radius(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "front_wheel_radius");
}

//-----------------------------------------------------------------------------
double get_front_hub_carrier_offset(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "front_hub_carrier_offset");
}

//-----------------------------------------------------------------------------
double get_rear_track(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "rear_track");
}

//-----------------------------------------------------------------------------
double get_rear_wheel_radius(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "rear_wheel_radius");
}

//-----------------------------------------------------------------------------
double get_rear_hub_carrier_offset(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "rear_hub_carrier_offset");
}

//-----------------------------------------------------------------------------
double get_idler_wheel_radius(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "idler_wheel_radius");
}


//-----------------------------------------------------------------------------
double get_roller_wheel_radius(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "roller_wheel_radius");
}

//-----------------------------------------------------------------------------
double get_sprocket_wheel_radius(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "sprocket_wheel_radius");
}

//-----------------------------------------------------------------------------
double get_track_thickness(const hardware_interface::HardwareInfo & hardware_info)
{
  return get_parameter<double>(hardware_info, "track_thickness");
}

}  // namespace ros2
}  // namespace romea
