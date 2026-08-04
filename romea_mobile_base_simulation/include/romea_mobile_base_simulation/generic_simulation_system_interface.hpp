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

#ifndef ROMEA_MOBILE_BASE_SIMULATION__GENERIC_SIMULATION_SYSTEM_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__GENERIC_SIMULATION_SYSTEM_INTERFACE_HPP_

// std
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// ros
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// romea
#include "romea_mobile_base_simulation/simulation_interface_base.hpp"

namespace romea
{
namespace ros2
{

class GenericSimulationSystemInterface : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  explicit GenericSimulationSystemInterface(
    const std::string & hardware_interface_name = "GenericSimulationSystemInterface");

  ~GenericSimulationSystemInterface() override = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

protected:
  virtual hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info);

  virtual hardware_interface::return_type load_interfaces_(
    const hardware_interface::HardwareInfo & hardware_info);

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void feedback_callback_(
    const std::string & interface_name, sensor_msgs::msg::JointState::ConstSharedPtr msg);

protected:
  std::string hardware_interface_name_;
  std::vector<std::string> simulation_interface_names_;
  std::unordered_map<std::string, std::unique_ptr<SimulationInterfaceBase>>
  simulation_interfaces_;

private:
  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr>
  joint_state_pubs_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
  joint_state_subs_;

  std::mutex mutex_;
  std::unordered_map<std::string, sensor_msgs::msg::JointState> feedbacks_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__GENERIC_SIMULATION_SYSTEM_INTERFACE_HPP_
