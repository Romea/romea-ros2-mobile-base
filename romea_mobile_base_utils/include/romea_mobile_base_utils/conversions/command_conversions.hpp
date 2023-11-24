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


#ifndef ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__COMMAND_CONVERSIONS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__COMMAND_CONVERSIONS_HPP_

// romea
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp"
#include "romea_mobile_base_msgs/msg/one_axle_steering_command.hpp"
#include "romea_mobile_base_msgs/msg/two_axle_steering_command.hpp"
#include "romea_mobile_base_msgs/msg/skid_steering_command.hpp"
#include "romea_mobile_base_msgs/msg/omni_steering_command.hpp"

// ros
#include "geometry_msgs/msg/twist.hpp"
#include "four_wheel_steering_msgs/msg/four_wheel_steering.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"

namespace romea
{
namespace ros2
{


void to_ros_msg(
  const core::TwoAxleSteeringCommand & romea_two_axle_steering_command,
  four_wheel_steering_msgs::msg::FourWheelSteering & ros_four_wheel_steering_msg);

void to_romea(
  const four_wheel_steering_msgs::msg::FourWheelSteering & ros_four_wheel_steering_msg,
  core::TwoAxleSteeringCommand & romea_two_axle_steering_command);

void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  ackermann_msgs::msg::AckermannDrive & ros_ackerman_drive_msg);

void to_romea(
  const ackermann_msgs::msg::AckermannDrive & ros_ackerman_drive_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command);

void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  geometry_msgs::msg::Twist & ros_twist_msg);

void to_romea(
  const geometry_msgs::msg::Twist & ros_twist_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command);

void to_ros_msg(
  const core::SkidSteeringCommand & romea_skid_steering_command,
  geometry_msgs::msg::Twist & ros_twist_msg);

void to_romea(
  const geometry_msgs::msg::Twist & ros_twist_msg,
  core::SkidSteeringCommand & romea_skid_steering_command);

void to_ros_msg(
  const core::OmniSteeringCommand & romea_omni_steering_command,
  geometry_msgs::msg::Twist & ros_twist_msg);

void to_romea(
  const geometry_msgs::msg::Twist & ros_twist_msg,
  core::OmniSteeringCommand & romea_omni_steering_command);

void to_ros_msg(
  const core::TwoAxleSteeringCommand & romea_two_axle_steering_command,
  romea_mobile_base_msgs::msg::TwoAxleSteeringCommand & ros_two_axle_steering_command_msg);

void to_romea(
  const romea_mobile_base_msgs::msg::TwoAxleSteeringCommand & ros_two_axle_steering_command_msg,
  core::TwoAxleSteeringCommand & romea_two_axle_steering_command);

void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  romea_mobile_base_msgs::msg::OneAxleSteeringCommand & ros_oxe_axle_steering_command_msg);

void to_romea(
  const romea_mobile_base_msgs::msg::OneAxleSteeringCommand & ros_one_axle_steering_command_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command);

void to_ros_msg(
  const core::SkidSteeringCommand & romea_skid_steering_command,
  romea_mobile_base_msgs::msg::SkidSteeringCommand & romea_skid_steering_command_msg);

void to_romea(
  const romea_mobile_base_msgs::msg::SkidSteeringCommand & ros_skid_steering_command_msg,
  core::SkidSteeringCommand & romea_skid_steering_command);

void to_ros_msg(
  const core::OmniSteeringCommand & romea_omni_steering_command,
  romea_mobile_base_msgs::msg::OmniSteeringCommand & romea_omni_steering_command_msg);

void to_romea(
  const romea_mobile_base_msgs::msg::OmniSteeringCommand & ros_omni_steering_command_msg,
  core::OmniSteeringCommand & romea_omni_steering_command);


}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONVERSIONS__COMMAND_CONVERSIONS_HPP_
