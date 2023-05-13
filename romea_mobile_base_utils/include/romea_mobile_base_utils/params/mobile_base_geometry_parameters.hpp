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


#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_

// std
#include <optional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_core_mobile_base/info/MobileBaseGeometry.hpp"
#include "romea_common_utils/params/node_parameters.hpp"


namespace romea
{

template<typename Node>
void declare_track_wheel_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "radius");
  declare_parameter<double>(node, parameters_ns, "x");
  declare_parameter_with_default<double>(
    node, parameters_ns, "z",
    std::numeric_limits<double>::quiet_NaN());
}

template<typename Node>
void try_declare_track_wheel_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  try {
    declare_track_wheel_info(node, parameters_ns);
  } catch (...) {
  }
}

template<typename Node>
void declare_track_idlers_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  try_declare_track_wheel_info(node, full_param_name(parameters_ns, "idler_wheel"));
  try_declare_track_wheel_info(node, full_param_name(parameters_ns, "front_idler_wheel"));
  try_declare_track_wheel_info(node, full_param_name(parameters_ns, "rear_idler_wheel"));
}

template<typename Node>
void declare_track_rollers_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "radius");
  declare_vector_parameter<double>(node, parameters_ns, "x");
  declare_parameter_with_default<double>(
    node, parameters_ns, "z",
    std::numeric_limits<double>::quiet_NaN());
}

template<typename Node>
TrackWheel get_track_wheel_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  auto radius = get_parameter<double>(node, parameters_ns, "radius");
  auto x = get_parameter<double>(node, parameters_ns, "x");
  auto z = get_parameter<double>(node, parameters_ns, "z");
  z = std::isfinite(z) ? z : radius;
  return {radius, x, z};
}

template<typename Node>
std::optional<TrackWheel> try_get_track_wheel_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  try {
    return get_track_wheel_info(node, parameters_ns);
  } catch (...) {
    return {};
  }
}


template<typename Node>
std::vector<TrackWheel> get_track_idlers_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  auto idler_wheel = try_get_track_wheel_info(
    node, full_param_name(parameters_ns, "idler_wheel"));

  if (idler_wheel) {
    return {*idler_wheel};
  }

  auto front_idler_wheel = try_get_track_wheel_info(
    node, full_param_name(parameters_ns, "front_idler_wheel"));
  auto rear_idler_wheel = try_get_track_wheel_info(
    node, full_param_name(parameters_ns, "rear_idler_wheel"));

  if (front_idler_wheel && rear_idler_wheel) {
    return {*front_idler_wheel, *rear_idler_wheel};
  }

  return {};
}

template<typename Node>
std::vector<TrackWheel> get_track_rollers_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  auto radius = get_parameter<double>(node, parameters_ns, "radius");
  auto x_vector = get_vector_parameter<double>(node, parameters_ns, "x");
  auto z = get_parameter<double>(node, parameters_ns, "z");
  z = std::isfinite(z) ? z : radius;

  std::vector<TrackWheel> roller_wheels;
  for (const double & x : x_vector) {
    TrackWheel wheel = {radius, x, z};
    roller_wheels.push_back(wheel);
  }
  return roller_wheels;
}


template<typename Node>
void declare_wheel_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "radius");
  declare_parameter<double>(node, parameters_ns, "width");
  declare_parameter_with_default<double>(node, parameters_ns, "hub_carrier_offset", 0.0);
}

template<typename Node>
Wheel get_wheel_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {get_parameter<double>(node, parameters_ns, "radius"),
    get_parameter<double>(node, parameters_ns, "width"),
    get_parameter<double>(node, parameters_ns, "hub_carrier_offset")};
}

template<typename Node>
void declare_continuous_track_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "width");
  declare_parameter<double>(node, parameters_ns, "thickness");
  declare_track_wheel_info(node, full_param_name(parameters_ns, "sprocket_wheel"));
  declare_track_idlers_info(node, parameters_ns);
  declare_track_rollers_info(node, full_param_name(parameters_ns, "rollers"));
}

template<typename Node>
ContinuousTrack get_continuous_track_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {get_parameter<double>(node, parameters_ns, "width"),
    get_parameter<double>(node, parameters_ns, "thickness"),
    get_track_wheel_info(node, full_param_name(parameters_ns, "sprocket_wheel")),
    get_track_idlers_info(node, parameters_ns),
    get_track_rollers_info(node, full_param_name(parameters_ns, "rollers"))};
}

template<typename Node>
void declare_wheeled_axle_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "wheels_distance");
  declare_wheel_info(node, full_param_name(parameters_ns, "wheels"));
}

template<typename Node>
WheeledAxle get_wheeled_axle_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {get_parameter<double>(node, parameters_ns, "wheels_distance"),
    get_wheel_info(node, full_param_name(parameters_ns, "wheels"))};
}

template<typename Node>
void declare_continuous_tracked_axle_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "tracks_distance");
  declare_continuous_track_info(node, full_param_name(parameters_ns, "tracks"));
}

template<typename Node>
ContinuousTrackedAxle get_continuous_tracked_axle_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {get_parameter<double>(node, parameters_ns, "tracks_distance"),
    get_continuous_track_info(node, full_param_name(parameters_ns, "tracks"))};
}

template<typename Node>
void declare_two_wheeled_axles_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "axles_distance");
  declare_wheeled_axle_info(node, full_param_name(parameters_ns, "front_axle"));
  declare_wheeled_axle_info(node, full_param_name(parameters_ns, "rear_axle"));
}

template<typename Node>
TwoWheeledAxles get_two_wheeled_axles_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {get_parameter<double>(node, parameters_ns, "axles_distance"),
    get_wheeled_axle_info(node, full_param_name(parameters_ns, "front_axle")),
    get_wheeled_axle_info(node, full_param_name(parameters_ns, "rear_axle"))};
}


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_GEOMETRY_PARAMETERS_HPP_
