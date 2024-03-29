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

// local
#include "romea_mobile_base_hardware/hardware_info.hpp"


namespace
{

const hardware_interface::InterfaceInfo &
get_interface_info(
  const std::vector<hardware_interface::InterfaceInfo> & interface_infos,
  const std::string & joint_name,
  const std::string & interface_type,
  const std::string & interface_name)
{
  const auto & interface = std::find_if(
    interface_infos.begin(),
    interface_infos.end(),
    [&interface_name](const auto & interface)
    {
      return interface.name == interface_name;
    });

  if (interface == interface_infos.cend()) {
    std::stringstream ss;
    ss << " Unable to obtain info of ";
    ss << interface_name;
    ss << " ";
    ss << interface_type;
    ss << " interface for joint ";
    ss << joint_name;
    throw(std::runtime_error(ss.str()));
  }

  return *interface;
}

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
const hardware_interface::InterfaceInfo &
get_command_interface_info(
  const hardware_interface::ComponentInfo & component_info,
  const std::string & interface_name)
{
  return get_interface_info(
    component_info.command_interfaces,
    component_info.name,
    "command",
    interface_name);
}

//-----------------------------------------------------------------------------
const hardware_interface::InterfaceInfo &
get_state_interface_info(
  const hardware_interface::ComponentInfo & component_info,
  const std::string & interface_name)
{
  return get_interface_info(
    component_info.state_interfaces,
    component_info.name,
    "state",
    interface_name);
}


//-----------------------------------------------------------------------------
bool has_parameter(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & parameter_name)
{
  return hardware_info.hardware_parameters.find(parameter_name) !=
         hardware_info.hardware_parameters.end();
}

//-----------------------------------------------------------------------------
const std::string & get_parameter(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & parameter_name)
{
  return hardware_info.hardware_parameters.at(parameter_name);
}

//-----------------------------------------------------------------------------
double get_min(const hardware_interface::InterfaceInfo & interface_info)
{
  if (!interface_info.min.empty()) {
    return std::stod(interface_info.min);
  } else {
    return -std::numeric_limits<double>::max();
  }
}

//-----------------------------------------------------------------------------
double get_max(const hardware_interface::InterfaceInfo & interface_info)
{
  if (!interface_info.max.empty()) {
    return std::stod(interface_info.min);
  } else {
    return std::numeric_limits<double>::max();
  }
}

}  // namespace ros2
}  // namespace romea
