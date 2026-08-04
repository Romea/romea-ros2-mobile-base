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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// local
// #include "romea_mobile_base_hardware/hardware_interface2WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FAS2FWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FAS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FAS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS2FWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS2FWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWC2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2TD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2THD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"
#include "romea_mobile_base_hardware/hardware_interface4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface4WS4WD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
std::vector<std::string> HardwareInterfaceBase::get_joint_names()
{
  return get_joint_state_command().name;
}

//-----------------------------------------------------------------------------
std::unique_ptr<HardwareInterfaceBase> make_hardware_interface(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & interface_type,
  const std::string & parameters_prefix)
{
  if (interface_type == "1FAS2FWD") {
    return std::make_unique<HardwareInterface1FAS2FWD>(
      HardwareInterface1FAS2FWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "1FAS2RWD") {
    return std::make_unique<HardwareInterface1FAS2RWD>(
      HardwareInterface1FAS2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "1FAS4WD") {
    return std::make_unique<HardwareInterface1FAS4WD>(
      HardwareInterface1FAS4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2AS2FWD") {
    return std::make_unique<HardwareInterface2AS2FWD>(
      HardwareInterface2AS2FWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2AS2RWD") {
    return std::make_unique<HardwareInterface2AS2RWD>(
      HardwareInterface2AS2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2AS4WD") {
    return std::make_unique<HardwareInterface2AS4WD>(
      HardwareInterface2AS4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS2FWD") {
    return std::make_unique<HardwareInterface2FWS2FWD>(
      HardwareInterface2FWS2FWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS2RWD") {
    return std::make_unique<HardwareInterface2FWS2RWD>(
      HardwareInterface2FWS2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWC2RWD") {
    return std::make_unique<HardwareInterface2FWC2RWD>(
      HardwareInterface2FWC2RWD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2FWS4WD") {
    return std::make_unique<HardwareInterface2FWS4WD>(
      HardwareInterface2FWS4WD::Configuration(hardware_info, parameters_prefix));
  }

  // if (interface_type == "2WD") {
  //   return std::make_unique<HardwareInterface2WD>(
  //     HardwareInterface2WD::Configuration(hardware_info, parameters_prefix));
  // }

  if (interface_type == "2TD") {
    return std::make_unique<HardwareInterface2TD>(
      HardwareInterface2TD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2THD") {
    return std::make_unique<HardwareInterface2THD>(
      HardwareInterface2THD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "2TTD") {
    return std::make_unique<HardwareInterface2TTD>(
      HardwareInterface2TTD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "4WD") {
    return std::make_unique<HardwareInterface4WD>(
      HardwareInterface4WD::Configuration(hardware_info, parameters_prefix));
  }

  if (interface_type == "4WS4WD") {
    return std::make_unique<HardwareInterface4WS4WD>(
      HardwareInterface4WS4WD::Configuration(hardware_info, parameters_prefix));
  }

  throw std::runtime_error("Unsupported hardware interface type: " + interface_type);
}

}  // namespace ros2
}  // namespace romea
