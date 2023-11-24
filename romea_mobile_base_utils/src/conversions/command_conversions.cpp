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


#include "romea_mobile_base_utils/conversions/command_conversions.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::TwoAxleSteeringCommand & romea_two_axle_steering_command,
  four_wheel_steering_msgs::msg::FourWheelSteering & ros_four_wheel_steering_msg)
{
  ros_four_wheel_steering_msg.front_steering_angle =
    romea_two_axle_steering_command.frontSteeringAngle;
  ros_four_wheel_steering_msg.rear_steering_angle =
    romea_two_axle_steering_command.rearSteeringAngle;
  ros_four_wheel_steering_msg.speed =
    romea_two_axle_steering_command.longitudinalSpeed;
}

//-----------------------------------------------------------------------------
void to_romea(
  const four_wheel_steering_msgs::msg::FourWheelSteering & ros_four_wheel_steering_msg,
  core::TwoAxleSteeringCommand & romea_two_axle_steering_command)
{
  romea_two_axle_steering_command.frontSteeringAngle =
    ros_four_wheel_steering_msg.front_steering_angle;
  romea_two_axle_steering_command.rearSteeringAngle =
    ros_four_wheel_steering_msg.rear_steering_angle;
  romea_two_axle_steering_command.longitudinalSpeed =
    ros_four_wheel_steering_msg.speed;
}


//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  ackermann_msgs::msg::AckermannDrive & ros_ackerman_drive_msg)
{
  ros_ackerman_drive_msg.speed = romea_one_axle_steering_command.longitudinalSpeed;
  ros_ackerman_drive_msg.steering_angle = romea_one_axle_steering_command.steeringAngle;
}

//-----------------------------------------------------------------------------
void to_romea(
  const ackermann_msgs::msg::AckermannDrive & ros_ackerman_drive_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command)
{
  romea_one_axle_steering_command.longitudinalSpeed = ros_ackerman_drive_msg.speed;
  romea_one_axle_steering_command.steeringAngle = ros_ackerman_drive_msg.steering_angle;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  geometry_msgs::msg::Twist & ros_twist_msg)
{
  ros_twist_msg.linear.x = romea_one_axle_steering_command.longitudinalSpeed;
  ros_twist_msg.angular.z = romea_one_axle_steering_command.steeringAngle;
}

//-----------------------------------------------------------------------------
void to_romea(
  const geometry_msgs::msg::Twist & ros_twist_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command)
{
  romea_one_axle_steering_command.longitudinalSpeed = ros_twist_msg.linear.x;
  romea_one_axle_steering_command.steeringAngle = ros_twist_msg.angular.z;
}


//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::SkidSteeringCommand & romea_skid_steering_command,
  geometry_msgs::msg::Twist & ros_twist_msg)
{
  ros_twist_msg.linear.x = romea_skid_steering_command.longitudinalSpeed;
  ros_twist_msg.angular.z = romea_skid_steering_command.angularSpeed;
}

//-----------------------------------------------------------------------------
void to_romea(
  const geometry_msgs::msg::Twist & ros_twist_msg,
  core::SkidSteeringCommand & romea_skid_steering_command)
{
  romea_skid_steering_command.longitudinalSpeed = ros_twist_msg.linear.x;
  romea_skid_steering_command.angularSpeed = ros_twist_msg.angular.z;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::OmniSteeringCommand & romea_omni_steering_command,
  geometry_msgs::msg::Twist & ros_twist_msg)
{
  ros_twist_msg.linear.x = romea_omni_steering_command.longitudinalSpeed;
  ros_twist_msg.linear.y = romea_omni_steering_command.lateralSpeed;
  ros_twist_msg.angular.z = romea_omni_steering_command.angularSpeed;
}

//-----------------------------------------------------------------------------
void to_romea(
  const geometry_msgs::msg::Twist & ros_twist_msg,
  core::OmniSteeringCommand & romea_omni_steering_command)
{
  romea_omni_steering_command.longitudinalSpeed = ros_twist_msg.linear.x;
  romea_omni_steering_command.lateralSpeed = ros_twist_msg.linear.y;
  romea_omni_steering_command.angularSpeed = ros_twist_msg.angular.z;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::TwoAxleSteeringCommand & romea_two_axle_steering_command,
  romea_mobile_base_msgs::msg::TwoAxleSteeringCommand & ros_two_axle_steering_command_msg)
{
  ros_two_axle_steering_command_msg.front_steering_angle =
    romea_two_axle_steering_command.frontSteeringAngle;
  ros_two_axle_steering_command_msg.rear_steering_angle =
    romea_two_axle_steering_command.rearSteeringAngle;
  ros_two_axle_steering_command_msg.longitudinal_speed =
    romea_two_axle_steering_command.longitudinalSpeed;
}

//-----------------------------------------------------------------------------
void to_romea(
  const romea_mobile_base_msgs::msg::TwoAxleSteeringCommand & ros_two_axle_steering_command_msg,
  core::TwoAxleSteeringCommand & romea_two_axle_steering_command)
{
  romea_two_axle_steering_command.frontSteeringAngle =
    ros_two_axle_steering_command_msg.front_steering_angle;
  romea_two_axle_steering_command.rearSteeringAngle =
    ros_two_axle_steering_command_msg.rear_steering_angle;
  romea_two_axle_steering_command.longitudinalSpeed =
    ros_two_axle_steering_command_msg.longitudinal_speed;
}


//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::OneAxleSteeringCommand & romea_one_axle_steering_command,
  romea_mobile_base_msgs::msg::OneAxleSteeringCommand & ros_oxe_axle_steering_command_msg)
{
  ros_oxe_axle_steering_command_msg.longitudinal_speed =
    romea_one_axle_steering_command.longitudinalSpeed;
  ros_oxe_axle_steering_command_msg.steering_angle = romea_one_axle_steering_command.steeringAngle;
}

//-----------------------------------------------------------------------------
void to_romea(
  const romea_mobile_base_msgs::msg::OneAxleSteeringCommand & ros_one_axle_steering_command_msg,
  core::OneAxleSteeringCommand & romea_one_axle_steering_command)
{
  romea_one_axle_steering_command.steeringAngle = ros_one_axle_steering_command_msg.steering_angle;
  romea_one_axle_steering_command.longitudinalSpeed =
    ros_one_axle_steering_command_msg.longitudinal_speed;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::SkidSteeringCommand & romea_skid_steering_command,
  romea_mobile_base_msgs::msg::SkidSteeringCommand & romea_skid_steering_command_msg)
{
  romea_skid_steering_command_msg.longitudinal_speed =
    romea_skid_steering_command.longitudinalSpeed;
  romea_skid_steering_command_msg.angular_speed =
    romea_skid_steering_command.angularSpeed;
}

//-----------------------------------------------------------------------------
void to_romea(
  const romea_mobile_base_msgs::msg::SkidSteeringCommand & ros_skid_steering_command_msg,
  core::SkidSteeringCommand & romea_skid_steering_command)
{
  romea_skid_steering_command.angularSpeed = ros_skid_steering_command_msg.angular_speed;
  romea_skid_steering_command.longitudinalSpeed = ros_skid_steering_command_msg.longitudinal_speed;
}


//-----------------------------------------------------------------------------
void to_ros_msg(
  const core::OmniSteeringCommand & romea_omni_steering_command,
  romea_mobile_base_msgs::msg::OmniSteeringCommand & romea_omni_steering_command_msg)
{
  romea_omni_steering_command_msg.longitudinal_speed =
    romea_omni_steering_command.longitudinalSpeed;
  romea_omni_steering_command_msg.lateral_speed =
    romea_omni_steering_command.lateralSpeed;
  romea_omni_steering_command_msg.angular_speed =
    romea_omni_steering_command.angularSpeed;
}

//-----------------------------------------------------------------------------
void to_romea(
  const romea_mobile_base_msgs::msg::OmniSteeringCommand & ros_omni_steering_command_msg,
  core::OmniSteeringCommand & romea_omni_steering_command)
{
  romea_omni_steering_command.angularSpeed = ros_omni_steering_command_msg.angular_speed;
  romea_omni_steering_command.lateralSpeed = ros_omni_steering_command_msg.lateral_speed;
  romea_omni_steering_command.longitudinalSpeed = ros_omni_steering_command_msg.longitudinal_speed;
}

}  // namespace ros2
}  // namespace romea
