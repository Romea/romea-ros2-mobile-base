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
// limitations under the License

// std
#include <map>
#include <string>
#include <iostream>

// local
#include "romea_teleop_drivers/two_axle_steering_teleop.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
TwoAxleSteeringTeleop::TwoAxleSteeringTeleop(const rclcpp::NodeOptions & options)
: TeleopBase(options),
  maximal_linear_speeds_(),
  maximal_front_steering_angle_(0),
  maximal_rear_steering_angle_(0),
  two_axes_linear_speed_control_(false),
  two_axes_steering_angle_control_(false),
  sent_disable_msg_(false)
{
  try {
    declare_parameters_();
    init_axes_control_modes_();
    init_joystick_();
    init_command_publisher_();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::declare_command_ranges_()
{
  declare_maximal_linear_speeds(node_);
  declare_maximal_front_steering_angle(node_);
  declare_maximal_rear_steering_angle(node_);
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::get_command_ranges_()
{
  maximal_linear_speeds_ = get_maximal_linear_speeds(node_);
  maximal_front_steering_angle_ = get_maximal_front_steering_angle(node_);
  maximal_rear_steering_angle_ = get_maximal_rear_steering_angle(node_);
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::declare_joystick_axes_mapping_()
{
  declare_forward_speed_axe_mapping(node_);
  declare_backward_speed_axe_mapping(node_);
  declare_front_steering_angle_axe_mapping(node_);
  declare_rear_steering_angle_axe_mapping(node_);
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::declare_joystick_buttons_mapping_()
{
  declare_slow_mode_button_mapping(node_);
  declare_turbo_mode_button_mapping(node_);
}

//-----------------------------------------------------------------------------
std::map<std::string, int> TwoAxleSteeringTeleop::get_joystick_axes_mapping_()
{
  return {
    {"forward_speed", get_forward_speed_axe_mapping(node_)},
    {"backward_speed", get_backward_speed_axe_mapping(node_)},
    {"front_steering_angle", get_front_steering_angle_axe_mapping(node_)},
    {"rear_steering_angle", get_rear_steering_angle_axe_mapping(node_)},
  };
}

//-----------------------------------------------------------------------------
std::map<std::string, int> TwoAxleSteeringTeleop::get_joystick_buttons_mapping_()
{
  return {
    {"slow_mode", get_slow_mode_button_mapping(node_)},
    {"turbo_mode", get_turbo_mode_button_mapping(node_)}
  };
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::init_axes_control_modes_()
{
  auto axes_mapping = get_joystick_axes_mapping_();
  two_axes_linear_speed_control_ =
    axes_mapping["forwad_speed"] != axes_mapping["backward_speed"];
  two_axes_steering_angle_control_ =
    axes_mapping["front_steering_angle"] != axes_mapping["rear_steering_angle"];
}

//-----------------------------------------------------------------------------
double TwoAxleSteeringTeleop::compute_linear_speed_(const double & maximal_linear_speed)const
{
  if (two_axes_steering_angle_control_) {
    return (joy_->getAxeValue("forward_speed") - joy_->getAxeValue("backward_speed")) *
           maximal_linear_speed / 2;
  } else {
    return (joy_->getAxeValue("forward_speed") + joy_->getAxeValue("backward_speed")) *
           maximal_linear_speed / 2;
  }
}

//-----------------------------------------------------------------------------
double TwoAxleSteeringTeleop::compute_front_steering_angle_() const
{
  return joy_->getAxeValue("front_steering_angle") * maximal_front_steering_angle_;
}

//-----------------------------------------------------------------------------
double TwoAxleSteeringTeleop::compute_rear_steering_angle_() const
{
  if (two_axes_steering_angle_control_) {
    return joy_->getAxeValue("rear_steering_angle") * maximal_rear_steering_angle_;
  } else {
    return -joy_->getAxeValue("rear_steering_angle") * maximal_rear_steering_angle_;
  }
}

//-----------------------------------------------------------------------------
void TwoAxleSteeringTeleop::joystick_callback_(const Joystick & joy)
{
  core::TwoAxleSteeringCommand cmd_msg;
  if (joy.getButtonValue("turbo_mode")) {
    cmd_msg.longitudinalSpeed = compute_linear_speed_(maximal_linear_speeds_.turbo_mode);
    cmd_msg.frontSteeringAngle = compute_front_steering_angle_();
    cmd_msg.rearSteeringAngle = compute_rear_steering_angle_();
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else if (joy.getButtonValue("slow_mode")) {
    cmd_msg.longitudinalSpeed = compute_linear_speed_(maximal_linear_speeds_.slow_mode);
    cmd_msg.frontSteeringAngle = compute_front_steering_angle_();
    cmd_msg.rearSteeringAngle = compute_rear_steering_angle_();
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else {
    // When mode button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg_) {
      cmd_pub_->publish(cmd_msg);
      sent_disable_msg_ = true;
    }
  }
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::TwoAxleSteeringTeleop)
