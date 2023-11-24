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
namespace ros2
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

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_INTERFACE_PARAMETERS_HPP_
