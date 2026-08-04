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
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_gazebo/gazebo_system_interface.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

namespace romea
{
namespace ros2
{

namespace
{

std::vector<std::string> split_interface_names(const std::string & interfaces)
{
  std::string normalized = interfaces;
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
GazeboSystemInterface::GazeboSystemInterface() : nh_(nullptr)
{
}

//-----------------------------------------------------------------------------
bool GazeboSystemInterface::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & enable_joints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & ecm,
  unsigned int /*update_rate*/)
{
  nh_ = model_nh;
  return load_interfaces_(ecm, enable_joints, hardware_info);
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GazeboSystemInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GazeboSystemInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GazeboSystemInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
bool GazeboSystemInterface::load_interfaces_(
  sim::EntityComponentManager & ecm,
  std::map<std::string, sim::Entity> & enable_joints,
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    const auto default_interface_names =
      get_parameter_or<std::string>(hardware_info, "hardware_interfaces", "mobile_base");

    interface_names_ = split_interface_names(
      get_parameter_or<std::string>(
        hardware_info, "simulation_interfaces", default_interface_names));

    if (interface_names_.empty()) {
      throw std::runtime_error("simulation_interfaces parameter is empty");
    }

    for (const auto & interface_name : interface_names_) {
      const auto interface_type = get_parameter(hardware_info, interface_name, "type");
      const auto parameters_prefix = get_parameter_or<std::string>(
        hardware_info, interface_name, "parameters_prefix", interface_name);

      auto simulation_interface =
        make_simulation_interface(hardware_info, interface_type, parameters_prefix);
      const auto gazebo_joint_infos =
        get_gazebo_joint_infos(hardware_info, interface_type, parameters_prefix);

      gazebo_interfaces_.emplace(
        interface_name,
        std::make_unique<GenericGazeboInterface>(ecm, enable_joints, gazebo_joint_infos));

      simulation_interfaces_.emplace(interface_name, std::move(simulation_interface));
    }

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), e.what());
    return false;
  }
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> GazeboSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & interface_name : interface_names_) {
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
GazeboSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & interface_name : interface_names_) {
    auto interface_command_interfaces =
      simulation_interfaces_.at(interface_name)->export_command_interfaces();
    command_interfaces.reserve(command_interfaces.size() + interface_command_interfaces.size());
    for (auto & command_interface : interface_command_interfaces) {
      command_interfaces.push_back(std::move(command_interface));
    }
  }
  return command_interfaces;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GazeboSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & interface_name : interface_names_) {
    const auto feedback = gazebo_interfaces_.at(interface_name)->get_joint_state();
    simulation_interfaces_.at(interface_name)->set_feedback(feedback);
  }
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GazeboSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & interface_name : interface_names_) {
    const auto command = simulation_interfaces_.at(interface_name)->get_joint_state_command();
    gazebo_interfaces_.at(interface_name)->set_command(command);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GazeboSystemInterface, gz_ros2_control::GazeboSimSystemInterface)
