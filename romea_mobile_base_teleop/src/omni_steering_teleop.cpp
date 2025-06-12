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
#include <map>
#include <string>

// local
#include "romea_teleop_drivers/omni_steering_teleop.hpp"


namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
OmniSteeringTeleop::OmniSteeringTeleop(const rclcpp::NodeOptions & options)
: TeleopBase(options),
  maximal_linear_speeds_(),
  maximal_lateral_speeds_(),
  maximal_angular_speeds_(),
  sent_disable_msg_(false)
{
  try {
    declare_parameters_();
    init_joystick_();
    init_command_publisher_();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
  }
}

//-----------------------------------------------------------------------------
void OmniSteeringTeleop::declare_command_ranges_()
{
  declare_maximal_linear_speeds(node_);
  declare_maximal_lateral_speeds(node_);
  declare_maximal_angular_speeds(node_);
}

//-----------------------------------------------------------------------------
void OmniSteeringTeleop::get_command_ranges_()
{
  maximal_linear_speeds_ = get_maximal_linear_speeds(node_);
  maximal_lateral_speeds_ = get_maximal_lateral_speeds(node_);
  maximal_angular_speeds_ = get_maximal_angular_speeds(node_);
}

//-----------------------------------------------------------------------------
void OmniSteeringTeleop::declare_joystick_axes_mapping_()
{
  declare_linear_speed_axe_mapping(node_);
  declare_lateral_speed_axe_mapping(node_);
  declare_angular_speed_axe_mapping(node_);
}

//-----------------------------------------------------------------------------
void OmniSteeringTeleop::declare_joystick_buttons_mapping_()
{
  declare_slow_mode_button_mapping(node_);
  declare_turbo_mode_button_mapping(node_);
}

//-----------------------------------------------------------------------------
std::map<std::string, int> OmniSteeringTeleop::get_joystick_axes_mapping_()
{
  return {
    {"linear_speed", get_linear_speed_axe_mapping(node_)},
    {"lateral_speed", get_lateral_speed_axe_mapping(node_)},
    {"angular_speed", get_angular_speed_axe_mapping(node_)},
  };
}

//-----------------------------------------------------------------------------
std::map<std::string, int> OmniSteeringTeleop::get_joystick_buttons_mapping_()
{
  return {
    {"slow_mode", get_slow_mode_button_mapping(node_)},
    {"turbo_mode", get_turbo_mode_button_mapping(node_)}
  };
}

//-----------------------------------------------------------------------------
double OmniSteeringTeleop::compute_linear_speed_(const double & maximal_linear_speed) const
{
  return joy_->getAxeValue("linear_speed") * maximal_linear_speed;
}

//-----------------------------------------------------------------------------
double OmniSteeringTeleop::compute_lateral_speed_(const double & maximal_lateral_speed) const
{
  return joy_->getAxeValue("lateral_speed") * maximal_lateral_speed;
}

//-----------------------------------------------------------------------------
double OmniSteeringTeleop::compute_angular_speed_(const double & maximal_angular_speed) const
{
  return joy_->getAxeValue("angular_speed") * maximal_angular_speed;
}


//-----------------------------------------------------------------------------
void OmniSteeringTeleop::joystick_callback_(const Joystick & joy)
{
  core::OmniSteeringCommand cmd_msg;
  if (joy.getButtonValue("turbo_mode")) {
    cmd_msg.longitudinalSpeed = compute_linear_speed_(maximal_linear_speeds_.turbo_mode);
    cmd_msg.lateralSpeed = compute_lateral_speed_(maximal_lateral_speeds_.turbo_mode);
    cmd_msg.angularSpeed = compute_angular_speed_(maximal_angular_speeds_.turbo_mode);
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else if (joy.getButtonValue("slow_mode")) {
    cmd_msg.longitudinalSpeed = compute_linear_speed_(maximal_linear_speeds_.slow_mode);
    cmd_msg.lateralSpeed = compute_lateral_speed_(maximal_lateral_speeds_.slow_mode);
    cmd_msg.angularSpeed = compute_angular_speed_(maximal_angular_speeds_.slow_mode);
    cmd_pub_->publish(cmd_msg);
    sent_disable_msg_ = false;
  } else {
    if (!sent_disable_msg_) {
      cmd_pub_->publish(cmd_msg);
      sent_disable_msg_ = true;
    }
  }
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::OmniSteeringTeleop)
