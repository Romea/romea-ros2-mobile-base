// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_INERTIA_PARAMETERS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_INERTIA_PARAMETERS_HPP_

// std
#include <memory>
#include <string>

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_core_mobile_base/info/MobileBaseInertia.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"


namespace romea
{

template<typename Node>
void declare_inertia_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  declare_parameter<double>(node, parameters_ns, "mass");

  declare_eigen_vector_parameter_with_default<Eigen::Vector3d>(
    node, parameters_ns, "center", Eigen::Vector3d::Zero());

  declare_parameter<double>(node, parameters_ns, "z_moment");
}

template<typename Node>
MobileBaseInertia get_inertia_info(
  std::shared_ptr<Node> node,
  const std::string & parameters_ns)
{
  return{get_parameter<double>(node, parameters_ns, "mass"),
    get_eigen_vector_parameter<Eigen::Vector3d>(node, parameters_ns, "center"),
    get_parameter<double>(node, parameters_ns, "z_moment")};
}

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__PARAMS__MOBILE_BASE_INERTIA_PARAMETERS_HPP_
