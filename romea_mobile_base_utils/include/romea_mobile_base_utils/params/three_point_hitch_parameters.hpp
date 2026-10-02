// Copyright 2026 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__THREE_POINT_HITCH_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__THREE_POINT_HITCH_PARAMETERS_HPP_

// std
#include <limits>
#include <memory>
#include <string>

// romea
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_core_mobile_base/info/ThreePointHitchInfo.hpp"

namespace romea
{
namespace ros2
{

template<typename Node>
void declare_three_point_hitch_info(
  std::shared_ptr<Node> node, const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "geometry.lower_link_length");
  declare_parameter<double>(node, parameters_ns, "geometry.lower_links_base_spacing");
  declare_parameter<double>(node, parameters_ns, "geometry.cylinders_x_offset");
  declare_parameter<double>(node, parameters_ns, "geometry.cylinders_z_offset");
  declare_parameter<double>(node, parameters_ns, "geometry.cylinders_base_spacing");
  declare_parameter<double>(node, parameters_ns, "geometry.cylinders_rod_attachment_distance");
  declare_parameter<double>(node, parameters_ns, "geometry.cylinders_rod_attachment_height");
  declare_parameter<double>(node, parameters_ns, "geometry.cylinders_dead_length");
  declare_parameter<double>(node, parameters_ns, "geometry.cylinders_stroke");
  declare_parameter<double>(node, parameters_ns, "geometry.upper_link_x_offset");
  declare_parameter<double>(node, parameters_ns, "geometry.upper_link_z_offset");
  declare_parameter<double>(node, parameters_ns, "geometry.upper_link_length");

  declare_parameter_with_default<double>(node, parameters_ns, "control.sensor.position_std", 0.0);
  declare_parameter<double>(node, parameters_ns, "control.command.minimal_length");
  declare_parameter<double>(node, parameters_ns, "control.command.maximal_length");
  declare_parameter<double>(node, parameters_ns, "control.command.maximal_speed");
  declare_parameter_with_default<double>(
    node, parameters_ns, "control.command.maximal_linear_acceleration",
    std::numeric_limits<double>::max());
}

template<typename Node>
core::ThreePointHitchInfo get_three_point_hitch_info(
  std::shared_ptr<Node> node, const std::string & parameters_ns)
{
  return {
    {
      get_parameter<double>(node, parameters_ns, "geometry.lower_link_length"),
      get_parameter<double>(node, parameters_ns, "geometry.lower_links_base_spacing"),
      get_parameter<double>(node, parameters_ns, "geometry.cylinders_x_offset"),
      get_parameter<double>(node, parameters_ns, "geometry.cylinders_z_offset"),
      get_parameter<double>(node, parameters_ns, "geometry.cylinders_base_spacing"),
      get_parameter<double>(node, parameters_ns, "geometry.cylinders_rod_attachment_distance"),
      get_parameter<double>(node, parameters_ns, "geometry.cylinders_rod_attachment_height"),
      get_parameter<double>(node, parameters_ns, "geometry.cylinders_dead_length"),
      get_parameter<double>(node, parameters_ns, "geometry.cylinders_stroke"),
      get_parameter<double>(node, parameters_ns, "geometry.upper_link_x_offset"),
      get_parameter<double>(node, parameters_ns, "geometry.upper_link_z_offset"),
      get_parameter<double>(node, parameters_ns, "geometry.upper_link_length")
    },
    {
      {get_parameter<double>(node, parameters_ns, "control.sensor.position_std")},
      {
        get_parameter<double>(node, parameters_ns, "control.command.minimal_length"),
        get_parameter<double>(node, parameters_ns, "control.command.maximal_length"),
        get_parameter<double>(node, parameters_ns, "control.command.maximal_speed"),
        get_parameter<double>(
          node, parameters_ns, "control.command.maximal_linear_acceleration")
      }
    }
  };
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__THREE_POINT_HITCH_PARAMETERS_HPP_
