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
#include <limits>
#include <map>
#include <memory>
#include <string>

// romea
#include "romea_common_utils/params/node_parameters.hpp"

// local
#include "romea_teleop_drivers/command_parameters.hpp"

namespace
{

const char COMMAND_RANGE_MAXIMAL_STEERING_ANGLE_PARAM_NAME[] =
  "cmd_range.maximal_steering_angle";
const char COMMAND_RANGE_MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME[] =
  "cmd_range.maximal_front_steering_angle";
const char COMMAND_RANGE_MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME[] =
  "cmd_range.maximal_rear_steering_angle";
const char COMMAND_RANGE_MAXIMAL_LINEAR_SPEED_PARAM_NAME[] =
  "cmd_range.maximal_linear_speed";
const char COMMAND_RANGE_MAXIMAL_LATERAL_SPEED_PARAM_NAME[] =
  "cmd_range.maximal_lateral_speed";
const char COMMAND_RANGE_MAXIMAL_ANGULAR_SPEED_PARAM_NAME[] =
  "cmd_range.maximal_angular_speed";

const char SLOW_MODE_SUFFIX[] = ".slow_mode";
const char TURBO_MODE_SUFFIX[] = ".turbo_mode";

const char COMMAND_MESSAGE_TYPE_PARAM_NAME[] = "cmd_output.message_type";
const char COMMAND_MESSAGE_PRIORITY_PARAM_NAME[] = "cmd_output.message_priority";


//-----------------------------------------------------------------------------
void declare_maximal_speeds(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & speed_param_name)
{
  romea::ros2::declare_parameter<double>(
    node, speed_param_name + SLOW_MODE_SUFFIX);

  romea::ros2::declare_parameter_with_default<double>(
    node, speed_param_name + TURBO_MODE_SUFFIX,
    std::numeric_limits<double>::quiet_NaN());
}

//-----------------------------------------------------------------------------
romea::ros2::MaximalSpeeds get_maximal_speeds(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & speed_param_name)
{
  romea::ros2::MaximalSpeeds maximal_speeds;

  maximal_speeds.slow_mode = romea::ros2::get_parameter<double>(
    node, speed_param_name + SLOW_MODE_SUFFIX);

  maximal_speeds.turbo_mode = romea::ros2::get_parameter<double>(
    node, speed_param_name + TURBO_MODE_SUFFIX);

  if (std::isnan(maximal_speeds.turbo_mode)) {
    maximal_speeds.turbo_mode = maximal_speeds.turbo_mode;
  }

  return maximal_speeds;
}

}  // namespace

namespace romea
{
namespace ros2
{


//-----------------------------------------------------------------------------
void declare_maximal_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<double>(node, COMMAND_RANGE_MAXIMAL_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_front_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<double>(node, COMMAND_RANGE_MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_rear_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<double>(node, COMMAND_RANGE_MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_linear_speeds(std::shared_ptr<rclcpp::Node> node)
{
  declare_maximal_speeds(node, COMMAND_RANGE_MAXIMAL_LINEAR_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_lateral_speeds(std::shared_ptr<rclcpp::Node> node)
{
  declare_maximal_speeds(node, COMMAND_RANGE_MAXIMAL_LATERAL_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_maximal_angular_speeds(std::shared_ptr<rclcpp::Node> node)
{
  declare_maximal_speeds(node, COMMAND_RANGE_MAXIMAL_ANGULAR_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
double get_maximal_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(node, COMMAND_RANGE_MAXIMAL_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
double get_maximal_front_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(node, COMMAND_RANGE_MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
double get_maximal_rear_steering_angle(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<double>(node, COMMAND_RANGE_MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
MaximalSpeeds get_maximal_linear_speeds(std::shared_ptr<rclcpp::Node> node)
{
  return get_maximal_speeds(node, COMMAND_RANGE_MAXIMAL_LINEAR_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
MaximalSpeeds get_maximal_lateral_speeds(std::shared_ptr<rclcpp::Node> node)
{
  return get_maximal_speeds(node, COMMAND_RANGE_MAXIMAL_LATERAL_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
MaximalSpeeds get_maximal_angular_speeds(std::shared_ptr<rclcpp::Node> node)
{
  return get_maximal_speeds(node, COMMAND_RANGE_MAXIMAL_ANGULAR_SPEED_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_command_output_message_type(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter<std::string>(node, COMMAND_MESSAGE_TYPE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
std::string get_command_output_message_type(std::shared_ptr<rclcpp::Node> node)
{
  //  Message type is not of the form package/type and cannot be processed
  return get_parameter<std::string>(node, COMMAND_MESSAGE_TYPE_PARAM_NAME);
}

//-----------------------------------------------------------------------------
void declare_command_output_message_priority(std::shared_ptr<rclcpp::Node> node)
{
  declare_parameter_with_default<int>(node, COMMAND_MESSAGE_PRIORITY_PARAM_NAME, -1);
}

//-----------------------------------------------------------------------------
int get_command_output_message_priority(std::shared_ptr<rclcpp::Node> node)
{
  return get_parameter<int>(node, COMMAND_MESSAGE_PRIORITY_PARAM_NAME);
}

}  // namespace ros2
}  // namespace romea
