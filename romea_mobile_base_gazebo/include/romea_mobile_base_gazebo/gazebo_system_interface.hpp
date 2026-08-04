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

#ifndef ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_SYSTEM_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_SYSTEM_INTERFACE_HPP_

// std
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ros
#include "gz_ros2_control/gz_system_interface.hpp"

// romea
#include "romea_mobile_base_simulation/simulation_interface_base.hpp"

// local
#include "romea_mobile_base_gazebo/generic_gazebo_interface.hpp"

namespace romea
{
namespace ros2
{

class GazeboSystemInterface : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  GazeboSystemInterface();

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & enable_joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & ecm,
    unsigned int update_rate) override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool load_interfaces_(
    sim::EntityComponentManager & ecm,
    std::map<std::string, sim::Entity> & enable_joints,
    const hardware_interface::HardwareInfo & hardware_info);

private:
  rclcpp::Node::SharedPtr nh_;
  std::vector<std::string> interface_names_;
  std::unordered_map<std::string, std::unique_ptr<SimulationInterfaceBase>> simulation_interfaces_;
  std::unordered_map<std::string, std::unique_ptr<GenericGazeboInterface>> gazebo_interfaces_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_SYSTEM_INTERFACE_HPP_
