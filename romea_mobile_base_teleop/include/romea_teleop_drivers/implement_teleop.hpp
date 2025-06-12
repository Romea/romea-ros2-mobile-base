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
#pragma once

// std
#include <map>
#include <string>
#include <memory>

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// romea
#include "romea_joystick_utils/joystick.hpp"

// local
#include "romea_teleop_drivers/command_parameters.hpp"
#include "romea_teleop_drivers/joystick_parameters.hpp"
#include "romea_teleop_drivers/visibility_control.h"

#ifndef ROMEA_TELEOP_DRIVERS__IMPLEMENT_TELEOP_HPP_
#define ROMEA_TELEOP_DRIVERS__IMPLEMENT_TELEOP_HPP_


namespace romea
{
namespace ros2
{

class ImplementTeleop
{
public:
  ROMEA_TELEOP_DRIVERS_PUBLIC
  explicit ImplementTeleop(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_DRIVERS_PUBLIC
  virtual ~ImplementTeleop();

  ROMEA_TELEOP_DRIVERS_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  void declare_parameters_();

  void init_joystick_();

  void joystick_callback_(const Joystick & joy);

  std::map<std::string, int> get_joystick_axes_mapping_();

  std::map<std::string, int> get_joystick_buttons_mapping_();

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<Joystick> joy_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  double increment_position_{0.05};
  std_msgs::msg::Float64MultiArray cmd_msg_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_TELEOP_DRIVERS__IMPLEMENT_TELEOP_HPP_
