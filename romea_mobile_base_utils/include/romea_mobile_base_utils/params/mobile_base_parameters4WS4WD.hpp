// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS4WS4WD_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS4WS4WD_HPP_

// std
#include <memory>
#include <string>

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_core_mobile_base/info/MobileBaseInfo4WS4WD.hpp"
#include "romea_mobile_base_utils/params/mobile_base_control_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"

namespace romea
{

template<typename Node>
void declare_mobile_base_info_4WS4WD(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_two_wheeled_axles_info(node, full_param_name(parameters_ns, "geometry"));
  declare_steering_angle_control_info(
    node, full_param_name(parameters_ns, "wheels_steering_control"));
  declare_wheel_speed_control_info(
    node,
    full_param_name(parameters_ns, "wheels_speed_control"));
  declare_inertia_info(node, full_param_name(parameters_ns, "inertia"));
  declare_eigen_vector_parameter<Eigen::Vector3d>(node, parameters_ns, "control_point");
}

template<typename Node>
MobileBaseInfo4WS4WD get_mobile_base_info_4WS4WD(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return {get_two_wheeled_axles_info(node, full_param_name(parameters_ns, "geometry")),
    get_steering_angle_control_info(
      node, full_param_name(parameters_ns, "wheels_steering_control")),
    get_wheel_speed_control_info(
      node, full_param_name(parameters_ns, "wheels_speed_control")),
    get_inertia_info(node, full_param_name(parameters_ns, "inertia")),
    get_eigen_vector_parameter<Eigen::Vector3d>(node, parameters_ns, "control_point")};
}


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_PARAMETERS4WS4WD_HPP_
