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

#ifndef ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE_BASE_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE_BASE_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// ros
#include "hardware_interface/hardware_info.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_interface_base.hpp"

namespace romea
{
namespace ros2
{

using SimulationInterfaceBase = HardwareInterfaceBase;

std::unique_ptr<SimulationInterfaceBase> make_simulation_interface(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & interface_type,
  const std::string & parameters_prefix);

hardware_interface::ComponentInfo make_gazebo_joint_info(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & command_interface_type);

std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & interface_type,
  const std::string & parameters_prefix);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE_BASE_HPP_
