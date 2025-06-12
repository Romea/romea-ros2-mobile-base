// Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
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
#include <map>
#include <algorithm>
#include <memory>
#include <utility>
#include <string>

// local
#include "romea_teleop_drivers/implement_teleop.hpp"

namespace romea
{
namespace ros2
{

ImplementTeleop::ImplementTeleop(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("implement_teleop", options)), joy_(nullptr), cmd_pub_(
    nullptr)
{
  try {
    declare_parameters_();
    init_joystick_();
    cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "position_controller/commands", 10);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
}

ImplementTeleop::~ImplementTeleop()
{
}

void ImplementTeleop::declare_parameters_()
{
  declare_up_down_implement_axe_mapping(node_);
  declare_down_implement_button_mapping(node_);
  declare_up_implement_button_mapping(node_);
}

void ImplementTeleop::init_joystick_()
{
  cmd_msg_.data.push_back(0.);
  auto axes_mapping = get_joystick_axes_mapping_();
  auto buttons_mapping = get_joystick_buttons_mapping_();
  joy_ = std::make_unique<Joystick>(node_, axes_mapping, buttons_mapping);

  auto callback = std::bind(&ImplementTeleop::joystick_callback_, this, std::placeholders::_1);
  joy_->registerOnReceivedMsgCallback(std::move(callback));
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ImplementTeleop::get_node_base_interface()
const
{
  return node_->get_node_base_interface();
}

std::map<std::string, int> ImplementTeleop::get_joystick_axes_mapping_()
{
  return {{"up_down_implement", get_up_down_implement_axe_mapping(node_)}};
}

std::map<std::string, int> ImplementTeleop::get_joystick_buttons_mapping_()
{
  return {{"down_implement", get_down_implement_button_mapping(node_)},
    {"up_implement", get_up_implement_button_mapping(node_)}};
}

void ImplementTeleop::joystick_callback_(const Joystick & joy)
{
  if (joy.getButtonValue("down_implement") || joy.getAxeValue("up_down_implement") < -0.5) {
    cmd_msg_.data[0] += increment_position_;
    cmd_msg_.data[0] = std::min(cmd_msg_.data[0], 1.);
    cmd_pub_->publish(cmd_msg_);
  } else if (joy.getButtonValue("up_implement") || joy.getAxeValue("up_down_implement") > 0.5) {
    cmd_msg_.data[0] -= increment_position_;
    cmd_msg_.data[0] = std::max(cmd_msg_.data[0], 0.);
    cmd_pub_->publish(cmd_msg_);
  }
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::ImplementTeleop)
