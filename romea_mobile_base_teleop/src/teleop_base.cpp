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
#include <utility>
#include <string>

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp"

// local
#include "romea_teleop_drivers/teleop_base.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<class CommandType>
TeleopBase<CommandType>::TeleopBase(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("teleop_node", options)),
  joy_(nullptr),
  cmd_pub_(nullptr),
  cmd_mux_client_(node_)
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
TeleopBase<CommandType>::~TeleopBase()
{
  try {
    cmd_mux_client_.unsubscribe(cmd_pub_->get_topic_name());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
}
//-----------------------------------------------------------------------------
template<class CommandType>
void TeleopBase<CommandType>::declare_parameters_()
{
  declare_command_output_message_type(node_);
  declare_command_output_message_priority(node_);
  declare_command_ranges_();

  declare_joystick_axes_mapping_();
  declare_joystick_buttons_mapping_();
}

//-----------------------------------------------------------------------------
template<class CommandType>
void TeleopBase<CommandType>::init_joystick_()
{
  auto axes_mapping = get_joystick_axes_mapping_();
  auto buttons_mapping = get_joystick_buttons_mapping_();
  joy_ = std::make_unique<Joystick>(node_, axes_mapping, buttons_mapping);

  auto callback = std::bind(&TeleopBase::joystick_callback_, this, std::placeholders::_1);
  joy_->registerOnReceivedMsgCallback(std::move(callback));
}

//-----------------------------------------------------------------------------
template<class CommandType>
void TeleopBase<CommandType>::init_command_publisher_()
{
  get_command_ranges_();
  int priority = get_command_output_message_priority(node_);
  std::string msg_type = get_command_output_message_type(node_);

  cmd_pub_ = make_command_publisher<CommandType>(node_, msg_type);
  cmd_pub_->activate();

  if (priority != -1) {
    cmd_mux_client_.subscribe(cmd_pub_->get_topic_name(), priority, 0.2);
  }
}


//-----------------------------------------------------------------------------
template<class CommandType>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
TeleopBase<CommandType>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

template class TeleopBase<core::SkidSteeringCommand>;
template class TeleopBase<core::OmniSteeringCommand>;
template class TeleopBase<core::OneAxleSteeringCommand>;
template class TeleopBase<core::TwoAxleSteeringCommand>;

}  // namespace ros2
}  // namespace romea
