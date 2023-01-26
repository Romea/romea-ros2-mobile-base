// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_INTERFACE_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_INTERFACE_PARAMETERS_HPP_

// std
#include <string>
#include <memory>

// romea
#include "romea_mobile_base_utils/control/command_interface.hpp"
#include "romea_common_utils/params/node_parameters.hpp"


namespace romea
{

template<typename Node>
void declare_command_interface_configuration(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<std::string>(node, parameters_ns, "message_type");
  declare_parameter_with_default<int>(node, parameters_ns, "priority", -1);
  declare_parameter<double>(node, parameters_ns, "rate");
}


//-----------------------------------------------------------------------------
template<typename Node>
CommandInterfaceConfiguration get_command_interface_configuration(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node, parameters_ns, "message_type"),
    get_parameter<int>(node, parameters_ns, "priority"),
    get_parameter<double>(node, parameters_ns, "rate")};
}

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_INTERFACE_PARAMETERS_HPP_
