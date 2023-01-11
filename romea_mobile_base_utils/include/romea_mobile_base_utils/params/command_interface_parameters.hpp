// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_INTERFACE_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_INTERFACE_PARAMETERS_HPP_

// std
#include <string>
#include <memory>

// romea
#include "romea_mobile_base_utils/control/command_interface.hpp"


namespace romea
{


void declare_command_interface_configuration(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);


CommandInterfaceConfiguration get_command_interface_configuration(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns);


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_INTERFACE_PARAMETERS_HPP_
