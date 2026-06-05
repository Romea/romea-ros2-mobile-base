// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_MOBILE_BASE_TELEOP__TELEOP_BASE_HPP_
#define ROMEA_MOBILE_BASE_TELEOP__TELEOP_BASE_HPP_

// std
#include <map>
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// romea
#include "romea_cmd_mux_utils/cmd_mux_interface.hpp"
#include "romea_joystick_utils/joystick.hpp"
#include "romea_mobile_base_utils/control/command_publisher.hpp"

// local
#include "romea_mobile_base_teleop/command_parameters.hpp"
#include "romea_mobile_base_teleop/joystick_parameters.hpp"
#include "romea_mobile_base_teleop/visibility_control.h"

namespace romea
{
namespace ros2
{

template<class CommandType>
class TeleopBase
{
public:
  using CmdPublisher = PublisherBase<CommandType>;

public:
  ROMEA_MOBILE_BASE_TELEOP_PUBLIC
  explicit TeleopBase(const rclcpp::NodeOptions & options);

  ROMEA_MOBILE_BASE_TELEOP_PUBLIC
  virtual ~TeleopBase();

  ROMEA_MOBILE_BASE_TELEOP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  void declare_parameters_();

  void init_command_publisher_();

  void init_joystick_();

  virtual void joystick_callback_(const Joystick & joy) = 0;

  virtual void declare_command_ranges_() = 0;

  virtual void get_command_ranges_() = 0;

  virtual void declare_joystick_axes_mapping_() = 0;

  virtual void declare_joystick_buttons_mapping_() = 0;

  virtual std::map<std::string, int> get_joystick_axes_mapping_() = 0;

  virtual std::map<std::string, int> get_joystick_buttons_mapping_() = 0;

protected:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<Joystick> joy_;
  std::shared_ptr<CmdPublisher> cmd_pub_;
  CmdMuxInterface cmd_mux_client_;
  bool is_connected_to_cmd_mux_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_TELEOP__TELEOP_BASE_HPP_
