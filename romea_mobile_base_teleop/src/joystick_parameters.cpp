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

// romea
#include "romea_common_utils/params/node_parameters.hpp"

// local
#include "romea_teleop_drivers/joystick_parameters.hpp"

namespace
{

const char FORWARD_SPEED_AXE_MAPPING_PARAM_NAME[] = "joystick_mapping.axes.forward_speed";
const char BACKWARD_SPEED_AXE_MAPPING_PARAM_NAME[] = "joystick_mapping.axes.backward_speed";
const char LINEAR_SPEED_AXE_MAPPING_PARAM_NAME[] = "joystick_mapping.axes.linear_speed";
const char LATERAL_SPEED_AXE_MAPPING_PARAM_NAME[] = "joystick_mapping.axes.lateral_speed";
const char ANGULAR_SPEED_AXE_MAPPING_PARAM_NAME[] = "joystick_mapping.axes.angular_speed";
const char STEERING_ANGLE_AXE_MAPPING_PARAM_NAME[] = "joystick_mapping.axes.steering_angle";
const char FRONT_STEERING_ANGLE_AXE_MAPPING_PARAM_NAME[] =
  "joystick_mapping.axes.front_steering_angle";
const char REAR_STEERING_ANGLE_AXE_MAPPING_PARAM_NAME[] =
  "joystick_mapping.axes.rear_steering_angle";
const char UP_DOWN_IMPLEMENT_AXE_MAPPING_PARAM_NAME[] = "joystick_mapping.axes.up_down_implement";
const char SLOW_MODE_BUTTON_MAPPING_PARAM_NAME[] = "joystick_mapping.buttons.slow_mode";
const char TURBO_MODE_BUTTON_MAPPING_PARAM_NAME[] = "joystick_mapping.buttons.turbo_mode";
const char DOWN_IMPLEMENT_BUTTON_MAPPING_PARAM_NAME[] = "joystick_mapping.buttons.down_implement";
const char UP_IMPLEMENT_BUTTON_MAPPING_PARAM_NAME[] = "joystick_mapping.buttons.up_implement";

}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void declare_forward_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, FORWARD_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_backward_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, BACKWARD_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_linear_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, LINEAR_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_lateral_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, LATERAL_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_angular_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, ANGULAR_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, STEERING_ANGLE_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_front_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, FRONT_STEERING_ANGLE_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_rear_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, REAR_STEERING_ANGLE_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_slow_mode_button_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, SLOW_MODE_BUTTON_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_turbo_mode_button_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<int>(node, TURBO_MODE_BUTTON_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_up_down_implement_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter_with_default<int>(node, UP_DOWN_IMPLEMENT_AXE_MAPPING_PARAM_NAME, -1);
}

//-----------------------------------------------------------------------------
void declare_down_implement_button_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter_with_default<int>(node, DOWN_IMPLEMENT_BUTTON_MAPPING_PARAM_NAME, -1);
}

//-----------------------------------------------------------------------------
void declare_up_implement_button_mapping(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter_with_default<int>(node, UP_IMPLEMENT_BUTTON_MAPPING_PARAM_NAME, -1);
}

//-----------------------------------------------------------------------------
int get_forward_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, FORWARD_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_backward_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, BACKWARD_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_linear_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, LINEAR_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_lateral_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, LATERAL_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_angular_speed_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, ANGULAR_SPEED_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, STEERING_ANGLE_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_front_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, FRONT_STEERING_ANGLE_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_rear_steering_angle_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, REAR_STEERING_ANGLE_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_slow_mode_button_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, SLOW_MODE_BUTTON_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_turbo_mode_button_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, TURBO_MODE_BUTTON_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_up_down_implement_axe_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, UP_DOWN_IMPLEMENT_AXE_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_down_implement_button_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, DOWN_IMPLEMENT_BUTTON_MAPPING_PARAM_NAME);
}

//-----------------------------------------------------------------------------
int get_up_implement_button_mapping(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, UP_IMPLEMENT_BUTTON_MAPPING_PARAM_NAME);
}

}  // namespace ros2
}  // namespace romea
