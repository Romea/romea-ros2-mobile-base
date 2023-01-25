// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>

// romea
#include "romea_common_utils/params/eigen_parameters.hpp"

// local
#include "romea_mobile_base_utils/params/mobile_base_parameters2WD.hpp"
#include "romea_mobile_base_utils/params/mobile_base_control_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"

namespace
{
const char geometry_param_name[] = "geometry";
const char wheels_speed_control_param_name[] = "wheels_speed_control";
const char control_point_param_name[] = "control_point";
const char inertia_param_name[] = "inertia";

const char left_wheel_spinning_joint_param_name[] = "left_wheel_spinning_joint_name";
const char right_wheel_spinning_joint_param_name[] = "right_wheel_spinning_joint_name";
}  // namespace

namespace romea
{

void declare_mobile_base_info_2WD(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns)
{
  declare_wheeled_axle_info(node, full_param_name(parameters_ns, geometry_param_name));
  declare_wheel_speed_control_info(
    node,
    full_param_name(parameters_ns, wheels_speed_control_param_name));
  declare_inertia_info(node, full_param_name(parameters_ns, inertia_param_name));
  declare_eigen_vector_parameter<Eigen::Vector3d>(node, parameters_ns, control_point_param_name);
}

MobileBaseInfo2WD get_mobile_base_info_2WD(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & parameters_ns)
{
  return {get_wheeled_axle_info(node, full_param_name(parameters_ns, geometry_param_name)),
    get_wheel_speed_control_info(
      node,
      full_param_name(parameters_ns, wheels_speed_control_param_name)),
    get_inertia_info(node, full_param_name(parameters_ns, inertia_param_name)),
    get_eigen_vector_parameter<Eigen::Vector3d>(node, parameters_ns, control_point_param_name)};
}

////-----------------------------------------------------------------------------
// void declare_joint_mappings_2WD(std::shared_ptr<rclcpp::Node> node,
//                                    const std::string & parameters_ns)
//{
//  declare_parameter<std::string>(node,parameters_ns,left_wheel_spinning_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,right_wheel_spinning_joint_param_name);
//}

////-----------------------------------------------------------------------------
// std::map<std::string,std::string> get_joint_mappings_2WD(std::shared_ptr<rclcpp::Node> node,
//                                                              const std::string & parameters_ns)
//{
//  std::map<std::string,std::string> joint_mappings;
//  insert_parameter_to_map(node,parameters_ns,left_wheel_spinning_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,right_wheel_spinning_joint_param_name,joint_mappings);
//  return joint_mappings;
//}

}  // namespace romea
