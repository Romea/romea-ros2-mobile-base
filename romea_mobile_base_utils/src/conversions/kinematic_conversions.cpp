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
#include <string>

// romea
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"

// local
#include "romea_mobile_base_utils/conversions/kinematic_conversions.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
KinematicMeasure to_romea(const romea_mobile_base_msgs::msg::KinematicMeasure & msg)
{
  KinematicMeasure kinematicMeasure;
  kinematicMeasure.longitudinalSpeed = msg.longitudinal_speed;
  kinematicMeasure.lateralSpeed = msg.lateral_speed;
  kinematicMeasure.angularSpeed = msg.angular_speed;
  kinematicMeasure.instantaneousCurvature = msg.instantaneous_curvature;
  kinematicMeasure.covariance = Eigen::Matrix4d(msg.covariance.data());
  return kinematicMeasure;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const KinematicMeasure & romea_kinematic_measure,
  romea_mobile_base_msgs::msg::KinematicMeasure & ros_kinematic_msg)
{
  ros_kinematic_msg.longitudinal_speed = romea_kinematic_measure.longitudinalSpeed;
  ros_kinematic_msg.lateral_speed = romea_kinematic_measure.lateralSpeed;
  ros_kinematic_msg.angular_speed = romea_kinematic_measure.angularSpeed;
  ros_kinematic_msg.instantaneous_curvature = romea_kinematic_measure.instantaneousCurvature;

  std::copy(
    romea_kinematic_measure.covariance.data(),
    romea_kinematic_measure.covariance.data() + 16,
    ros_kinematic_msg.covariance.data());
}


//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const KinematicMeasure & romea_kinematic_measure,
  romea_mobile_base_msgs::msg::KinematicMeasureStamped & ros_kinematic_stamped_msg)
{
  ros_kinematic_stamped_msg.header.frame_id = frame_id;
  ros_kinematic_stamped_msg.header.stamp = stamp;
  to_ros_msg(romea_kinematic_measure, ros_kinematic_stamped_msg.measure);
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const KinematicMeasure & romea_kinematic_measure,
  geometry_msgs::msg::TwistWithCovariance & ros_twist_with_covariance)
{
  ros_twist_with_covariance.twist.linear.x = romea_kinematic_measure.longitudinalSpeed;
  ros_twist_with_covariance.twist.linear.y = romea_kinematic_measure.lateralSpeed;
  ros_twist_with_covariance.twist.angular.z = romea_kinematic_measure.angularSpeed;
  ros_twist_with_covariance.covariance[0] = romea_kinematic_measure.covariance(0, 0);
  ros_twist_with_covariance.covariance[4] = romea_kinematic_measure.covariance(0, 1);
  ros_twist_with_covariance.covariance[5] = romea_kinematic_measure.covariance(0, 2);
  ros_twist_with_covariance.covariance[6] = romea_kinematic_measure.covariance(1, 0);
  ros_twist_with_covariance.covariance[7] = romea_kinematic_measure.covariance(1, 1);
  ros_twist_with_covariance.covariance[11] = romea_kinematic_measure.covariance(1, 2);
  ros_twist_with_covariance.covariance[30] = romea_kinematic_measure.covariance(2, 0);
  ros_twist_with_covariance.covariance[31] = romea_kinematic_measure.covariance(2, 1);
  ros_twist_with_covariance.covariance[35] = romea_kinematic_measure.covariance(2, 2);
}


//-----------------------------------------------------------------------------
OneAxleSteeringMeasure to_romea(const romea_mobile_base_msgs::msg::OneAxleSteeringMeasure & msg)
{
  OneAxleSteeringMeasure oneAxleSteeringMeasure;
  oneAxleSteeringMeasure.longitudinalSpeed = msg.longitudinal_speed;
  oneAxleSteeringMeasure.steeringAngle = msg.steering_angle;
  oneAxleSteeringMeasure.covariance = Eigen::Matrix2d(msg.covariance.data());
  return oneAxleSteeringMeasure;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const OneAxleSteeringMeasure & romea_one_axle_steering_measure,
  romea_mobile_base_msgs::msg::OneAxleSteeringMeasure & ros_one_axle_steering_measure_msg)
{
  ros_one_axle_steering_measure_msg.longitudinal_speed =
    romea_one_axle_steering_measure.longitudinalSpeed;
  ros_one_axle_steering_measure_msg.steering_angle = romea_one_axle_steering_measure.steeringAngle;

  std::copy(
    ros_one_axle_steering_measure_msg.covariance.data(),
    ros_one_axle_steering_measure_msg.covariance.data() + 4,
    ros_one_axle_steering_measure_msg.covariance.data());
}


//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const OneAxleSteeringMeasure & romea_one_axle_steering_measure,
  romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped & ros_one_axle_steering_measure_msg)
{
  ros_one_axle_steering_measure_msg.header.frame_id = frame_id;
  ros_one_axle_steering_measure_msg.header.stamp = stamp;
  to_ros_msg(romea_one_axle_steering_measure, ros_one_axle_steering_measure_msg.measure);
}


//-----------------------------------------------------------------------------
TwoAxleSteeringMeasure to_romea(const romea_mobile_base_msgs::msg::TwoAxleSteeringMeasure & msg)
{
  TwoAxleSteeringMeasure twoAxleSteeringMeasure;
  twoAxleSteeringMeasure.longitudinalSpeed = msg.longitudinal_speed;
  twoAxleSteeringMeasure.frontSteeringAngle = msg.front_steering_angle;
  twoAxleSteeringMeasure.rearSteeringAngle = msg.rear_steering_angle;
  twoAxleSteeringMeasure.covariance = Eigen::Matrix3d(msg.covariance.data());
  return twoAxleSteeringMeasure;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const TwoAxleSteeringMeasure & romea_two_axle_steering_measure,
  romea_mobile_base_msgs::msg::TwoAxleSteeringMeasure & ros_two_axle_steering_measure_msg)
{
  ros_two_axle_steering_measure_msg.longitudinal_speed =
    romea_two_axle_steering_measure.longitudinalSpeed;
  ros_two_axle_steering_measure_msg.front_steering_angle =
    romea_two_axle_steering_measure.frontSteeringAngle;
  ros_two_axle_steering_measure_msg.rear_steering_angle =
    romea_two_axle_steering_measure.rearSteeringAngle;

  std::copy(
    ros_two_axle_steering_measure_msg.covariance.data(),
    ros_two_axle_steering_measure_msg.covariance.data() + 9,
    ros_two_axle_steering_measure_msg.covariance.data());
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const TwoAxleSteeringMeasure & romea_two_axle_steering_measure,
  romea_mobile_base_msgs::msg::TwoAxleSteeringMeasureStamped & ros_two_axle_steering_measure_msg)
{
  ros_two_axle_steering_measure_msg.header.frame_id = frame_id;
  ros_two_axle_steering_measure_msg.header.stamp = stamp;
  to_ros_msg(romea_two_axle_steering_measure, ros_two_axle_steering_measure_msg.measure);
}

//-----------------------------------------------------------------------------
SkidSteeringMeasure to_romea(const romea_mobile_base_msgs::msg::SkidSteeringMeasure & msg)
{
  SkidSteeringMeasure skidSteeringMeasure;
  skidSteeringMeasure.longitudinalSpeed = msg.longitudinal_speed;
  skidSteeringMeasure.angularSpeed = msg.angular_speed;
  skidSteeringMeasure.covariance = Eigen::Matrix2d(msg.covariance.data());
  return skidSteeringMeasure;
}

//-----------------------------------------------------------------------------
OmniSteeringMeasure to_romea(const romea_mobile_base_msgs::msg::OmniSteeringMeasure & msg)
{
  OmniSteeringMeasure omniSteeringMeasure;
  omniSteeringMeasure.longitudinalSpeed = msg.longitudinal_speed;
  omniSteeringMeasure.lateralSpeed = msg.lateral_speed;
  omniSteeringMeasure.angularSpeed = msg.angular_speed;
  omniSteeringMeasure.covariance = Eigen::Matrix3d(msg.covariance.data());
  return omniSteeringMeasure;
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const OmniSteeringMeasure & romea_omni_steering_measure,
  romea_mobile_base_msgs::msg::OmniSteeringMeasure & ros_omni_steering_measure_msg)
{
  ros_omni_steering_measure_msg.longitudinal_speed = romea_omni_steering_measure.longitudinalSpeed;
  ros_omni_steering_measure_msg.lateral_speed = romea_omni_steering_measure.lateralSpeed;
  ros_omni_steering_measure_msg.angular_speed = romea_omni_steering_measure.angularSpeed;

  std::copy(
    ros_omni_steering_measure_msg.covariance.data(),
    ros_omni_steering_measure_msg.covariance.data() + 9,
    ros_omni_steering_measure_msg.covariance.data());
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const OmniSteeringMeasure & romea_omni_steering_measure,
  romea_mobile_base_msgs::msg::OmniSteeringMeasureStamped & ros_omni_steering_measure_msg)
{
  ros_omni_steering_measure_msg.header.frame_id = frame_id;
  ros_omni_steering_measure_msg.header.stamp = stamp;
  to_ros_msg(romea_omni_steering_measure, ros_omni_steering_measure_msg.measure);
}


//-----------------------------------------------------------------------------
void to_ros_msg(
  const SkidSteeringMeasure & romea_skid_steering_measure,
  romea_mobile_base_msgs::msg::SkidSteeringMeasure & ros_skid_steering_measure_msg)
{
  ros_skid_steering_measure_msg.longitudinal_speed = romea_skid_steering_measure.longitudinalSpeed;
  ros_skid_steering_measure_msg.angular_speed = romea_skid_steering_measure.angularSpeed;

  std::copy(
    ros_skid_steering_measure_msg.covariance.data(),
    ros_skid_steering_measure_msg.covariance.data() + 4,
    ros_skid_steering_measure_msg.covariance.data());
}

//-----------------------------------------------------------------------------
void to_ros_msg(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const SkidSteeringMeasure & romea_skid_steering_measure,
  romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped & ros_skid_steering_measure_msg)
{
  ros_skid_steering_measure_msg.header.frame_id = frame_id;
  ros_skid_steering_measure_msg.header.stamp = stamp;
  to_ros_msg(romea_skid_steering_measure, ros_skid_steering_measure_msg.measure);
}

}  // namespace romea
