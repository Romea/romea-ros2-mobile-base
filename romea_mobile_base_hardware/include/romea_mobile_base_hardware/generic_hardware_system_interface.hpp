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

#ifndef ROMEA_MOBILE_BASE_HARDWARE__GENERIC_HARDWARE_SYSTEM_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__GENERIC_HARDWARE_SYSTEM_INTERFACE_HPP_

// std
#include <mutex>
#include <string>
#include <unordered_map>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"

namespace romea
{
namespace ros2
{

class GenericHardwareSystemInterface : public HardwareSystemInterface
{
public:
  explicit GenericHardwareSystemInterface(
    const std::string & hardware_interface_name = "GenericHardwareSystemInterface");

  ~GenericHardwareSystemInterface() override = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

protected:
  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void feedback_callback_(
    const std::string & interface_name, sensor_msgs::msg::JointState::ConstSharedPtr msg);

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

#endif  // ROMEA_MOBILE_BASE_HARDWARE__GENERIC_HARDWARE_SYSTEM_INTERFACE_HPP_
