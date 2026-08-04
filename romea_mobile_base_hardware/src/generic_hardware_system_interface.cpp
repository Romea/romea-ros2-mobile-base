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
#include <string>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_mobile_base_hardware/generic_hardware_system_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
GenericHardwareSystemInterface::GenericHardwareSystemInterface(
  const std::string & hardware_interface_name)
: HardwareSystemInterface(hardware_interface_name),
  node_(nullptr)
{
}

//-----------------------------------------------------------------------------
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericHardwareSystemInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  const auto callback_return = HardwareSystemInterface::on_init(hardware_info);
  if (callback_return != CallbackReturn::SUCCESS) {
    return callback_return;
  }

  node_ = std::make_shared<rclcpp::Node>("joint_state_bridge");

  for (const auto & interface_name : hardware_interface_names_) {
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
hardware_interface::return_type GenericHardwareSystemInterface::connect_()
{
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GenericHardwareSystemInterface::disconnect_()
{
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GenericHardwareSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);

  std::lock_guard<std::mutex> guard(mutex_);
  for (const auto & [interface_name, feedback] : feedbacks_) {
    hardware_interfaces_.at(interface_name)->set_feedback(feedback);
  }

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type GenericHardwareSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & interface_name : hardware_interface_names_) {
    joint_state_pubs_.at(interface_name)->publish(
      hardware_interfaces_.at(interface_name)->get_joint_state_command());
  }

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void GenericHardwareSystemInterface::feedback_callback_(
  const std::string & interface_name, sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> guard(mutex_);
  feedbacks_[interface_name] = *msg;
}

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GenericHardwareSystemInterface, hardware_interface::SystemInterface)
