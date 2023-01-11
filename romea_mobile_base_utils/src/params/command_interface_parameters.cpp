// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_common_utils/params/node_parameters.hpp>

// std
#include <memory>
#include <string>

// local
#include "romea_mobile_base_utils/control/command_interface.hpp"

namespace
{
const char OUTPUT_MESSAGE_TYPE_PARAM_NAME[] = "message_type";
const char PRIORITY_PARAM_NAME[] = "priority";
const char RATE_PARAM_NAME[] = "rate";
}

namespace romea
{

//-----------------------------------------------------------------------------
void declare_command_interface_configuration(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<std::string>(node, parameters_ns, OUTPUT_MESSAGE_TYPE_PARAM_NAME);
  declare_parameter_with_default<int>(node, parameters_ns, PRIORITY_PARAM_NAME, -1);
  declare_parameter<double>(node, parameters_ns, RATE_PARAM_NAME);
}


//-----------------------------------------------------------------------------
CommandInterfaceConfiguration get_command_interface_configuration(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node, parameters_ns, OUTPUT_MESSAGE_TYPE_PARAM_NAME),
    get_parameter<int>(node, parameters_ns, PRIORITY_PARAM_NAME),
    get_parameter<double>(node, parameters_ns, RATE_PARAM_NAME)};
}

}  // namespace romea
