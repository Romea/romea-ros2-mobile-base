// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_

// ros
#include <rclcpp/node.hpp>

// romea
#include <romea_core_mobile_base/info/MobileBaseControl.hpp>

// std
#include <memory>
#include <string>

namespace romea
{

void declare_steering_angle_control_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

void declare_wheel_speed_control_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

SteeringAngleControl get_steering_angle_control_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

WheelSpeedControl get_wheel_speed_control_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_
