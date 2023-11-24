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


#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_LIMITS_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_LIMITS_PARAMETERS_HPP_

// std
#include <memory>
#include <string>
#include <limits>

// romea
#include "romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommandLimits.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommandLimits.hpp"
#include "romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp"

// romea ros
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{
namespace ros2
{

template<typename Node>
void declare_minimal_longitudinal_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter_with_default<double>(
    node, parameters_ns, "minimal_longitudinal_speed", -std::numeric_limits<double>::max());
}

template<typename Node>
void declare_maximal_longitudinal_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_longitudinal_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
void declare_maximal_lateral_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_lateral_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
void declare_maximal_angular_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_angular_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
void declare_maximal_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_steering_angle", M_PI_2);
}

template<typename Node>
void declare_maximal_front_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_front_steering_angle", M_PI_2);
}

template<typename Node>
void declare_maximal_rear_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_rear_steering_angle", M_PI_2);
}

template<typename Node>
void declare_one_axle_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_minimal_longitudinal_speed(node, parameters_ns);
  declare_maximal_longitudinal_speed(node, parameters_ns);
  declare_maximal_steering_angle(node, parameters_ns);
}

template<typename Node>
void declare_skid_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_minimal_longitudinal_speed(node, parameters_ns);
  declare_maximal_longitudinal_speed(node, parameters_ns);
  declare_maximal_angular_speed(node, parameters_ns);
}

template<typename Node>
void declare_two_axle_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_minimal_longitudinal_speed(node, parameters_ns);
  declare_maximal_longitudinal_speed(node, parameters_ns);
  declare_maximal_front_steering_angle(node, parameters_ns);
  declare_maximal_rear_steering_angle(node, parameters_ns);
}

template<typename Node>
void declare_omni_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_minimal_longitudinal_speed(node, parameters_ns);
  declare_maximal_longitudinal_speed(node, parameters_ns);
  declare_maximal_lateral_speed(node, parameters_ns);
  declare_maximal_angular_speed(node, parameters_ns);
}

template<typename Node>
double get_minimal_longitudinal_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return get_parameter_or<double>(
    node, parameters_ns, "minimal_longitudinal_speed",
    -std::numeric_limits<double>::max());
}

template<typename Node>
double get_maximal_longitudinal_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return get_parameter_or<double>(
    node, parameters_ns, "maximal_longitudinal_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
double get_maximal_lateral_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return get_parameter_or<double>(
    node, parameters_ns, "maximal_lateral_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
double get_maximal_angular_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return get_parameter_or<double>(
    node, parameters_ns, "maximal_angular_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
double get_maximal_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return get_parameter_or<double>(
    node, parameters_ns, "maximal_steering_angle", M_PI_2);
}

template<typename Node>
double get_maximal_front_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return get_parameter_or<double>(
    node, parameters_ns, "maximal_front_steering_angle", M_PI_2);
}

template<typename Node>
double get_maximal_rear_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return get_parameter_or<double>(
    node, parameters_ns, "maximal_rear_steering_angle", M_PI_2);
}


template<typename Node>
core::SkidSteeringCommandLimits get_skid_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return core::SkidSteeringCommandLimits(
    get_minimal_longitudinal_speed(node, parameters_ns),
    get_maximal_longitudinal_speed(node, parameters_ns),
    get_maximal_angular_speed(node, parameters_ns));
}

template<typename Node>
core::OmniSteeringCommandLimits get_omni_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return core::OmniSteeringCommandLimits(
    get_minimal_longitudinal_speed(node, parameters_ns),
    get_maximal_longitudinal_speed(node, parameters_ns),
    get_maximal_lateral_speed(node, parameters_ns),
    get_maximal_angular_speed(node, parameters_ns));
}


template<typename Node>
core::OneAxleSteeringCommandLimits get_one_axle_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return core::OneAxleSteeringCommandLimits(
    get_minimal_longitudinal_speed(node, parameters_ns),
    get_maximal_longitudinal_speed(node, parameters_ns),
    get_maximal_steering_angle(node, parameters_ns));
}

template<typename Node>
core::TwoAxleSteeringCommandLimits get_two_axle_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return core::TwoAxleSteeringCommandLimits(
    get_minimal_longitudinal_speed(node, parameters_ns),
    get_maximal_longitudinal_speed(node, parameters_ns),
    get_maximal_front_steering_angle(node, parameters_ns),
    get_maximal_rear_steering_angle(node, parameters_ns));
}


template<typename Limits, typename Node>
void declare_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  if constexpr (std::is_same_v<Limits, core::SkidSteeringCommandLimits>) {
    declare_skid_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, core::OmniSteeringCommandLimits>) {
    declare_omni_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, core::OneAxleSteeringCommandLimits>) {
    declare_one_axle_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, core::TwoAxleSteeringCommandLimits>) {
    declare_two_axle_steering_command_limits(node, parameters_ns);
  }
}


template<typename Limits, typename Node>
Limits get_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  if constexpr (std::is_same_v<Limits, core::SkidSteeringCommandLimits>) {
    return get_skid_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, core::OmniSteeringCommandLimits>) {
    return get_omni_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, core::OneAxleSteeringCommandLimits>) {
    return get_one_axle_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, core::TwoAxleSteeringCommandLimits>) {
    return get_two_axle_steering_command_limits(node, parameters_ns);
  }
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_LIMITS_PARAMETERS_HPP_
