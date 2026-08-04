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
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_mobile_base_simulation/generic_simulation_system_interface.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

namespace romea
{
namespace ros2
{

namespace
{

std::vector<std::string> split_interface_names(const std::string & simulation_interfaces)
{
  std::string normalized = simulation_interfaces;
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
GenericSimulationSystemInterface::GenericSimulationSystemInterface(
  const std::string & hardware_interface_name)
: hardware_interface_name_(hardware_interface_name), node_(nullptr)
{
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (
    load_info_(hardware_info) != hardware_interface::return_type::OK ||
    load_interfaces_(hardware_info) != hardware_interface::return_type::OK) {
    return CallbackReturn::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>("simulation_joint_state_bridge");

  for (const auto & interface_name : simulation_interface_names_) {
    joint_state_pubs_[interface_name] = node_->create_publisher<sensor_msgs::msg::JointState>(
      "bridge/" + interface_name + "/joint_state_command", sensor_data_qos());

    joint_state_subs_[interface_name] = node_->create_subscription<sensor_msgs::msg::JointState>(
      "bridge/" + interface_name + "/joint_state_feedback",
      best_effort(1),
      [this, interface_name](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        feedback_callback_(interface_name, msg);
      });
  }

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GenericSimulationSystemInterface::load_info_(
  const hardware_interface::HardwareInfo & /*hardware_info*/)
{
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GenericSimulationSystemInterface::load_interfaces_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    const auto default_interface_names =
      get_parameter_or<std::string>(hardware_info, "hardware_interfaces", "mobile_base");

    simulation_interface_names_ = split_interface_names(
      get_parameter_or<std::string>(
        hardware_info, "simulation_interfaces", default_interface_names));

    if (simulation_interface_names_.empty()) {
      throw std::runtime_error("simulation_interfaces parameter is empty");
    }

    for (const auto & interface_name : simulation_interface_names_) {
      const auto interface_type = get_parameter(hardware_info, interface_name, "type");
      const auto parameters_prefix = get_parameter_or<std::string>(
        hardware_info, interface_name, "parameters_prefix", interface_name);

      simulation_interfaces_.emplace(
        interface_name,
        make_simulation_interface(hardware_info, interface_type, parameters_prefix));
    }

    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger(hardware_interface_name_), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_configure : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_cleanup : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_activate : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_deactivate : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_shutdown : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface::on_error(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_error : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GenericSimulationSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);

  std::lock_guard<std::mutex> guard(mutex_);
  for (const auto & [interface_name, feedback] : feedbacks_) {
    simulation_interfaces_.at(interface_name)->set_feedback(feedback);
  }

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GenericSimulationSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & interface_name : simulation_interface_names_) {
    joint_state_pubs_.at(interface_name)
      ->publish(simulation_interfaces_.at(interface_name)->get_joint_state_command());
  }

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void GenericSimulationSystemInterface::feedback_callback_(
  const std::string & interface_name, sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> guard(mutex_);
  feedbacks_[interface_name] = *msg;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
GenericSimulationSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & interface_name : simulation_interface_names_) {
    auto interface_state_interfaces =
      simulation_interfaces_.at(interface_name)->export_state_interfaces();
    state_interfaces.reserve(state_interfaces.size() + interface_state_interfaces.size());
    for (auto & state_interface : interface_state_interfaces) {
      state_interfaces.push_back(std::move(state_interface));
    }
  }
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
GenericSimulationSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & interface_name : simulation_interface_names_) {
    auto interface_command_interfaces =
      simulation_interfaces_.at(interface_name)->export_command_interfaces();
    command_interfaces.reserve(command_interfaces.size() + interface_command_interfaces.size());
    for (auto & command_interface : interface_command_interfaces) {
      command_interfaces.push_back(std::move(command_interface));
    }
  }
  return command_interfaces;
}

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GenericSimulationSystemInterface, hardware_interface::SystemInterface)
