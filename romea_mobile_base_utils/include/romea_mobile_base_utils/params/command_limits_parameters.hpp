// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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

template<typename Node>
void declare_minimal_longitudinal_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(
    node, parameters_ns, "minimal_longitudinal_speed", -std::numeric_limits<double>::max());
}

template<typename Node>
void declare_maximal_longitudinal_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_longitudinal_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
void declare_maximal_lateral_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_lateral_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
void declare_maximal_angular_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_angular_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
void declare_maximal_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_steering_angle", M_PI_2);
}

template<typename Node>
void declare_maximal_front_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(
    node, parameters_ns, "maximal_front_steering_angle", M_PI_2);
}

template<typename Node>
void declare_maximal_rear_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(
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
  return romea::get_parameter_or<double>(
    node, parameters_ns, "minimal_longitudinal_speed",
    -std::numeric_limits<double>::max());
}

template<typename Node>
double get_maximal_longitudinal_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return romea::get_parameter_or<double>(
    node, parameters_ns, "maximal_longitudinal_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
double get_maximal_lateral_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return romea::get_parameter_or<double>(
    node, parameters_ns, "maximal_lateral_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
double get_maximal_angular_speed(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return romea::get_parameter_or<double>(
    node, parameters_ns, "maximal_angular_speed",
    std::numeric_limits<double>::max());
}

template<typename Node>
double get_maximal_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return romea::get_parameter_or<double>(
    node, parameters_ns, "maximal_steering_angle", M_PI_2);
}

template<typename Node>
double get_maximal_front_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return romea::get_parameter_or<double>(
    node, parameters_ns, "maximal_front_steering_angle", M_PI_2);
}

template<typename Node>
double get_maximal_rear_steering_angle(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return romea::get_parameter_or<double>(
    node, parameters_ns, "maximal_rear_steering_angle", M_PI_2);
}


template<typename Node>
SkidSteeringCommandLimits get_skid_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return SkidSteeringCommandLimits(
    get_minimal_longitudinal_speed(node, parameters_ns),
    get_maximal_longitudinal_speed(node, parameters_ns),
    get_maximal_angular_speed(node, parameters_ns));
}

template<typename Node>
OmniSteeringCommandLimits get_omni_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return OmniSteeringCommandLimits(
    get_minimal_longitudinal_speed(node, parameters_ns),
    get_maximal_longitudinal_speed(node, parameters_ns),
    get_maximal_lateral_speed(node, parameters_ns),
    get_maximal_angular_speed(node, parameters_ns));
}


template<typename Node>
OneAxleSteeringCommandLimits get_one_axle_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return OneAxleSteeringCommandLimits(
    get_minimal_longitudinal_speed(node, parameters_ns),
    get_maximal_longitudinal_speed(node, parameters_ns),
    get_maximal_steering_angle(node, parameters_ns));
}

template<typename Node>
TwoAxleSteeringCommandLimits get_two_axle_steering_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return TwoAxleSteeringCommandLimits(
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
  if constexpr (std::is_same_v<Limits, SkidSteeringCommandLimits>) {
    declare_skid_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, OmniSteeringCommandLimits>) {
    declare_omni_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, OneAxleSteeringCommandLimits>) {
    declare_one_axle_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, TwoAxleSteeringCommandLimits>) {
    declare_two_axle_steering_command_limits(node, parameters_ns);
  }
}


template<typename Limits, typename Node>
Limits get_command_limits(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  if constexpr (std::is_same_v<Limits, SkidSteeringCommandLimits>) {
    return get_skid_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, OmniSteeringCommandLimits>) {
    return get_omni_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, OneAxleSteeringCommandLimits>) {
    return get_one_axle_steering_command_limits(node, parameters_ns);
  } else if constexpr (std::is_same_v<Limits, TwoAxleSteeringCommandLimits>) {
    return get_two_axle_steering_command_limits(node, parameters_ns);
  }
}

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__COMMAND_LIMITS_PARAMETERS_HPP_
