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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// romea
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "romea_mobile_base_simulation/simulation_interface1FAS2FWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface1FAS2RWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2AS4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWC2RWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWS2FWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWS2RWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWS4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2TD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2THD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2TTD.hpp"
#include "romea_mobile_base_simulation/simulation_interface4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface4WS4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface_base.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo make_gazebo_joint_info(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & command_interface_type)
{
  auto gazebo_joint_info = joint_info;
  gazebo_joint_info.command_interfaces.clear();

  if (!command_interface_type.empty()) {
    const auto command_interface = std::find_if(
      joint_info.command_interfaces.begin(),
      joint_info.command_interfaces.end(),
      [&command_interface_type](const auto & interface_info) {
        return interface_info.name == command_interface_type;
      });

    if (command_interface != joint_info.command_interfaces.end()) {
      gazebo_joint_info.command_interfaces.push_back(*command_interface);
    } else {
      hardware_interface::InterfaceInfo interface_info;
      interface_info.name = command_interface_type;
      gazebo_joint_info.command_interfaces.push_back(interface_info);
    }
  }

  return gazebo_joint_info;
}

//-----------------------------------------------------------------------------
std::unique_ptr<SimulationInterfaceBase> make_simulation_interface(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & interface_type,
  const std::string & parameters_prefix)
{
  if (interface_type == "1FAS2FWD") {
    return std::make_unique<SimulationInterface1FAS2FWD>(
      SimulationInterface1FAS2FWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "1FAS2RWD") {
    return std::make_unique<SimulationInterface1FAS2RWD>(
      SimulationInterface1FAS2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2AS4WD") {
    return std::make_unique<SimulationInterface2AS4WD>(
      SimulationInterface2AS4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS2FWD") {
    return std::make_unique<SimulationInterface2FWS2FWD>(
      SimulationInterface2FWS2FWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS2RWD") {
    return std::make_unique<SimulationInterface2FWS2RWD>(
      SimulationInterface2FWS2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWC2RWD") {
    return std::make_unique<SimulationInterface2FWC2RWD>(
      SimulationInterface2FWC2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS4WD") {
    return std::make_unique<SimulationInterface2FWS4WD>(
      HardwareInterface2FWS4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2TD") {
    return std::make_unique<SimulationInterface2TD>(
      SimulationInterface2TD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2THD") {
    return std::make_unique<SimulationInterface2THD>(
      SimulationInterface2THD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2TTD") {
    return std::make_unique<SimulationInterface2TTD>(
      SimulationInterface2TTD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "4WD") {
    return std::make_unique<SimulationInterface4WD>(
      HardwareInterface4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "4WS4WD") {
    return std::make_unique<SimulationInterface4WS4WD>(
      HardwareInterface4WS4WD::Configuration(hardware_info, parameters_prefix));
  }

  throw std::runtime_error("Unsupported simulation interface type: " + interface_type);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & interface_type,
  const std::string & parameters_prefix)
{
  if (interface_type == "1FAS2FWD") {
    return get_gazebo_joint_infos(
      SimulationInterface1FAS2FWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "1FAS2RWD") {
    return get_gazebo_joint_infos(
      SimulationInterface1FAS2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2AS4WD") {
    return get_gazebo_joint_infos(
      SimulationInterface2AS4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS2FWD") {
    return get_gazebo_joint_infos(
      SimulationInterface2FWS2FWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS2RWD") {
    return get_gazebo_joint_infos(
      SimulationInterface2FWS2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWC2RWD") {
    return get_gazebo_joint_infos(
      SimulationInterface2FWC2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS4WD") {
    return get_gazebo_joint_infos(
      HardwareInterface2FWS4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2TD") {
    return get_gazebo_joint_infos(
      SimulationInterface2TD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2THD") {
    return get_gazebo_joint_infos(
      SimulationInterface2THD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2TTD") {
    return get_gazebo_joint_infos(
      SimulationInterface2TTD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "4WD") {
    return get_gazebo_joint_infos(
      HardwareInterface4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "4WS4WD") {
    return get_gazebo_joint_infos(
      HardwareInterface4WS4WD::Configuration(hardware_info, parameters_prefix));
  }

  throw std::runtime_error("Unsupported simulation interface type: " + interface_type);
}

}  // namespace ros2
}  // namespace romea
