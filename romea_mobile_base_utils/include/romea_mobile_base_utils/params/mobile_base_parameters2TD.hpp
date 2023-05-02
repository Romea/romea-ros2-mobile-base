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


#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2TD_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2TD_HPP_

// std
#include <string>
#include <memory>

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_core_mobile_base/info/MobileBaseInfo2TD.hpp"
#include "romea_mobile_base_utils/params/mobile_base_control_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"

namespace romea
{

template<typename Node>
void declare_mobile_base_info_2TD(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_continuous_tracked_axle_info(
    node, full_param_name(parameters_ns, "geometry"));
  declare_wheel_speed_control_info(
    node, full_param_name(parameters_ns, "tracks_speed_control"));
  declare_inertia_info(node, full_param_name(parameters_ns, "inertia"));
  declare_eigen_vector_parameter<Eigen::Vector3d>(node, parameters_ns, "control_point");
}

//-----------------------------------------------------------------------------
template<typename Node>
MobileBaseInfo2TD get_mobile_base_info_2TD(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {get_continuous_tracked_axle_info(
      node, full_param_name(parameters_ns, "geometry")),
    get_wheel_speed_control_info(
      node, full_param_name(parameters_ns, "tracks_speed_control")),
    get_inertia_info(node, full_param_name(parameters_ns, "inertia")),
    get_eigen_vector_parameter<Eigen::Vector3d>(node, parameters_ns, "control_point")};
}


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS2TD_HPP_
