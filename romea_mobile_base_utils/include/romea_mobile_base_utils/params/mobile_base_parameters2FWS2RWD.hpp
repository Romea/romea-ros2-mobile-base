// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2FWS2RWD_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2FWS2RWD_HPP_

// std
#include <memory>
#include <string>

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_core_mobile_base/info/MobileBaseInfo2FWS2RWD.hpp"


namespace romea
{

void declare_mobile_base_info_2FWS2RWD(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

MobileBaseInfo2FWS2RWD get_mobile_base_info_2FWS2RWD(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);

// void declare_joint_mappings_2FWS2RWD(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & parameters_ns);

// std::map<std::string, std::string> get_joint_mappings_2FWS2RWD(
//   std::shared_ptr<rclcpp::Node> node,
//   const std::string & parameters_ns);
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2FWS2RWD_HPP_
