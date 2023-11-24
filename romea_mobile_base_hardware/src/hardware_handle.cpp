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
#include <vector>
#include <string>

// local
#include "romea_mobile_base_hardware/hardware_handle.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HardwareCommandInterface::HardwareCommandInterface(
  const hardware_interface::InterfaceInfo & interface_info,
  const std::string & joint_name)
: command_(0.0),
  command_min_(get_min(interface_info)),
  command_max_(get_max(interface_info)),
  joint_name_(joint_name),
  interface_type_(interface_info.name)
{
}

//-----------------------------------------------------------------------------
HardwareCommandInterface::HardwareCommandInterface(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & interface_type)
: HardwareCommandInterface(get_command_interface_info(joint_info, interface_type), joint_info.name)
{
}

//-----------------------------------------------------------------------------
double HardwareCommandInterface::get() const
{
  return command_;
}

//-----------------------------------------------------------------------------
void HardwareCommandInterface::export_interface(
  std::vector<hardware_interface::CommandInterface> & hardware_interfaces)
{
  using hardware_interface::CommandInterface;
  hardware_interfaces.push_back(CommandInterface(joint_name_, interface_type_, &command_));
}


//-----------------------------------------------------------------------------
const std::string & HardwareCommandInterface::get_interface_type() const
{
  return interface_type_;
}

//-----------------------------------------------------------------------------
const std::string & HardwareCommandInterface::get_joint_name() const
{
  return joint_name_;
}

//-----------------------------------------------------------------------------
HardwareStateInterface::HardwareStateInterface(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & interface_type)
: HardwareStateInterface(get_state_interface_info(joint_info, interface_type), joint_info.name)
{
}

//-----------------------------------------------------------------------------
HardwareStateInterface::HardwareStateInterface(
  const hardware_interface::InterfaceInfo & interface_info,
  const std::string & joint_name)
: state_(0.0),
  state_min_(get_min(interface_info)),
  state_max_(get_max(interface_info)),
  joint_name_(joint_name),
  interface_type_(interface_info.name)
{
}


//-----------------------------------------------------------------------------
void HardwareStateInterface::set(const double & state)
{
  state_ = state;
}

//-----------------------------------------------------------------------------
void HardwareStateInterface::export_interface(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  using hardware_interface::StateInterface;
  state_interfaces.push_back(StateInterface(joint_name_, interface_type_, &state_));
}

//-----------------------------------------------------------------------------
const std::string & HardwareStateInterface::get_interface_type() const
{
  return interface_type_;
}

//-----------------------------------------------------------------------------
const std::string & HardwareStateInterface::get_joint_name() const
{
  return joint_name_;
}

}  // namespace ros2
}  // namespace romea
