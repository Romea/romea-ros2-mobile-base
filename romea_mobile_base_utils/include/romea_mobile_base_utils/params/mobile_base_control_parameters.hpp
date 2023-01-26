// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_

// std
#include <limits>
#include <memory>
#include <string>

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_core_mobile_base/info/MobileBaseControl.hpp"
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{

template<typename Node>
void declare_steering_angle_control_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "sensor.angle_std");

  declare_parameter_with_default<double>(
    node, parameters_ns, "sensor.angle_range",
    std::numeric_limits<double>::max());

  declare_parameter<double>(node, parameters_ns, "command.maximal_angle");
  declare_parameter<double>(node, parameters_ns, "command.maximal_angular_speed");
}

template<typename Node>
SteeringAngleControl get_steering_angle_control_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {{get_parameter<double>(node, parameters_ns, "sensor.angle_std"),
    get_parameter<double>(node, parameters_ns, "sensor.angle_range")},
    {get_parameter<double>(node, parameters_ns, "command.maximal_angle"),
      get_parameter<double>(node, parameters_ns, "command.maximal_angular_speed")}};
}

template<typename Node>
void declare_wheel_speed_control_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "sensor.speed_std");

  declare_parameter_with_default<double>(
    node, parameters_ns, "sensor.speed_range",
    std::numeric_limits<double>::max());

  declare_parameter<double>(node, parameters_ns, "command.maximal_speed");
  declare_parameter<double>(node, parameters_ns, "command.maximal_acceleration");
}

template<typename Node>
WheelSpeedControl get_wheel_speed_control_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {{get_parameter<double>(node, parameters_ns, "sensor.speed_std"),
    get_parameter<double>(node, parameters_ns, "sensor.speed_range")},
    {get_parameter<double>(node, parameters_ns, "command.maximal_speed"),
      get_parameter<double>(node, parameters_ns, "command.maximal_acceleration")}};
}

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_CONTROL_PARAMETERS_HPP_
