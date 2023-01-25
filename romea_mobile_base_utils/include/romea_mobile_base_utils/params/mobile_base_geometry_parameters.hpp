// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_

// std
#include <memory>
#include <string>

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"


namespace romea
{

void declare_wheel_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

void declare_wheeled_axle_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

void declare_two_wheeled_axles_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

void declare_continuous_track_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

void declare_continuous_tracked_axle_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

Wheel get_wheel_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

WheeledAxle get_wheeled_axle_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

TwoWheeledAxles get_two_wheeled_axles_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

ContinuousTrack get_continuous_track_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

ContinuousTrackedAxle get_continuous_tracked_axle_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_
