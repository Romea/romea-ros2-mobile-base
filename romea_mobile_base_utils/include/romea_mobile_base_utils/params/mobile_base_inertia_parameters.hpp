// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_INERTIA_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_INERTIA_PARAMETERS_HPP_

// ros
#include <rclcpp/node.hpp>

// romea
#include <romea_core_mobile_base/info/MobileBaseInertia.hpp>

// std
#include <memory>
#include <string>

namespace romea
{

void declare_inertia_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

MobileBaseInertia get_inertia_info(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_INERTIA_PARAMETERS_HPP_
