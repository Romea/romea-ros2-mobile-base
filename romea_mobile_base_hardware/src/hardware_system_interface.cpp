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
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_interface_base.hpp"
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

namespace romea
{
namespace ros2
{

namespace
{

std::vector<std::string> split_interface_names(const std::string & hardware_interfaces)
{
  std::string normalized = hardware_interfaces;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::replace(normalized.begin(), normalized.end(), ';', ' ');

  std::stringstream stream(normalized);
  std::vector<std::string> names;
  std::string name;
  while (stream >> name) {
    names.push_back(name);
  }
  return names;
}

}  // namespace

//-----------------------------------------------------------------------------
HardwareSystemInterface::HardwareSystemInterface(
  const std::string & hardware_interface_name)
: hardware_interface_name_(hardware_interface_name)
{
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareSystemInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (
    load_info_(hardware_info) == hardware_interface::return_type::OK &&
    load_interfaces_(hardware_info) == hardware_interface::return_type::OK) {
    return CallbackReturn::SUCCESS;
  } else {
    return CallbackReturn::ERROR;
  }
}

//-----------------------------------------------------------------------------
hardware_interface::return_type HardwareSystemInterface::load_info_(
  const hardware_interface::HardwareInfo & /*hardware_info*/)
{
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type HardwareSystemInterface::load_interfaces_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    hardware_interface_names_ = split_interface_names(
      get_parameter_or<std::string>(hardware_info, "hardware_interfaces", "mobile_base"));

    if (hardware_interface_names_.empty()) {
      throw std::runtime_error("hardware_interfaces parameter is empty");
    }

    for (const auto & interface_name : hardware_interface_names_) {
      const auto interface_type = get_parameter(hardware_info, interface_name, "type");
      const auto parameters_prefix = get_parameter_or<std::string>(
        hardware_info, interface_name, "parameters_prefix", interface_name);

      hardware_interfaces_.emplace(
        interface_name,
        make_hardware_interface(hardware_info, interface_type, parameters_prefix));
    }

    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger(hardware_interface_name_), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareSystemInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_configure : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareSystemInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_cleanup : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareSystemInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_activate : previous state " << int(previous_state.id()) << " " << previous_state.label());

  return connect_() == hardware_interface::return_type::OK ?
         CallbackReturn::SUCCESS :
         CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareSystemInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_deactivate : previous state " << int(previous_state.id()) << " " << previous_state.label());

  return disconnect_() == hardware_interface::return_type::OK ?
         CallbackReturn::SUCCESS :
         CallbackReturn::ERROR;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareSystemInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_shutdown : previous state " << int(previous_state.id()) << " " << previous_state.label());

  if (static_cast<int>(previous_state.id()) == 1) {
    return CallbackReturn::SUCCESS;
  }

  return disconnect_() == hardware_interface::return_type::OK ?
         CallbackReturn::SUCCESS :
         CallbackReturn::ERROR;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareSystemInterface::on_error(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_error : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> HardwareSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & interface_name : hardware_interface_names_) {
    auto interface_state_interfaces =
      hardware_interfaces_.at(interface_name)->export_state_interfaces();
    state_interfaces.reserve(state_interfaces.size() + interface_state_interfaces.size());
    for (auto & state_interface : interface_state_interfaces) {
      state_interfaces.push_back(std::move(state_interface));
    }
  }
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & interface_name : hardware_interface_names_) {
    auto interface_command_interfaces =
      hardware_interfaces_.at(interface_name)->export_command_interfaces();
    command_interfaces.reserve(command_interfaces.size() + interface_command_interfaces.size());
    for (auto & command_interface : interface_command_interfaces) {
      command_interfaces.push_back(std::move(command_interface));
    }
  }
  return command_interfaces;
}

//-----------------------------------------------------------------------------
HardwareInterfaceBase & HardwareSystemInterface::hardware_interface_(
  const std::string & interface_name)
{
  return *hardware_interfaces_.at(interface_name);
}

//-----------------------------------------------------------------------------
const HardwareInterfaceBase & HardwareSystemInterface::hardware_interface_(
  const std::string & interface_name) const
{
  return *hardware_interfaces_.at(interface_name);
}

}  // namespace ros2
}  // namespace romea
