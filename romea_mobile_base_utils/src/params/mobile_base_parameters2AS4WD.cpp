#include "romea_mobile_base_utils/params/mobile_base_parameters2AS4WD.hpp"
#include "romea_mobile_base_utils/params/mobile_base_control_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"
#include <romea_common_utils/params/eigen_parameters.hpp>

namespace  {
const std::string geometry_param_name = "geometry";
const std::string axles_steering_control_param_name = "axles_steering_control";
const std::string wheels_speed_control_param_name = "wheels_speed_control";
const std::string control_point_param_name="control_point";
const std::string inertia_param_name="inertia";

const std::string front_axle_steering_joint_param_name="front_axle_steering_joint_name";
const std::string rear_axle_steering_joint_param_name="rear_axle_steering_joint_name";
const std::string front_left_wheel_steering_joint_param_name="front_left_wheel_steering_joint_name";
const std::string front_right_wheel_steering_joint_param_name="front_right_wheel_steering_joint_name";
const std::string rear_left_wheel_steering_joint_param_name="rear_left_wheel_steering_joint_name";
const std::string rear_right_wheel_steering_joint_param_name="rear_right_wheel_steering_joint_name";
const std::string front_left_wheel_spinning_joint_param_name="front_left_wheel_spinning_joint_name";
const std::string front_right_wheel_spinning_joint_param_name="front_right_wheel_spinning_joint_name";
const std::string rear_left_wheel_spinning_joint_param_name="rear_left_wheel_spinning_joint_name";
const std::string rear_right_wheel_spinning_joint_param_name="rear_right_wheel_spinning_joint_name";

}

namespace romea {

//-----------------------------------------------------------------------------
void declare_mobile_base_info_2AS4WD(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & parameters_ns)
{
  declare_two_wheeled_axles_info(
        node,full_param_name(parameters_ns,geometry_param_name));

  declare_steering_angle_control_info(
        node,full_param_name(parameters_ns,axles_steering_control_param_name));

  declare_wheel_speed_control_info(
        node,full_param_name(parameters_ns,wheels_speed_control_param_name));

  declare_inertia_info(
        node,full_param_name(parameters_ns,inertia_param_name));

  declare_eigen_vector_parameter<Eigen::Vector3d>(
        node,parameters_ns,control_point_param_name);
}

//-----------------------------------------------------------------------------
MobileBaseInfo2AS4WD get_mobile_base_info_2AS4WD(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string & parameters_ns)
{
  return {get_two_wheeled_axles_info(node,full_param_name(parameters_ns,geometry_param_name)),
        get_steering_angle_control_info(node,full_param_name(parameters_ns,axles_steering_control_param_name)),
        get_wheel_speed_control_info(node,full_param_name(parameters_ns,wheels_speed_control_param_name)),
        get_inertia_info(node,full_param_name(parameters_ns,inertia_param_name)),
        get_eigen_vector_parameter<Eigen::Vector3d>(node,parameters_ns,control_point_param_name)};

}


////-----------------------------------------------------------------------------
//void declare_joint_mappings_2AS4WD(std::shared_ptr<rclcpp::Node> node,
//                                   const std::string & parameters_ns)
//{
//  declare_parameter<std::string>(node,parameters_ns,front_axle_steering_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,rear_axle_steering_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,rear_left_wheel_steering_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,rear_right_wheel_steering_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name);
//  declare_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name);

//}

////-----------------------------------------------------------------------------
//std::map<std::string,std::string> get_joint_mappings_2AS4WD(std::shared_ptr<rclcpp::Node> node,
//                                                            const std::string & parameters_ns)
//{
//  std::map<std::string,std::string> joint_mappings;
//  insert_parameter_to_map(node,parameters_ns,front_axle_steering_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,rear_axle_steering_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,front_left_wheel_steering_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,front_right_wheel_steering_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,rear_left_wheel_steering_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,rear_right_wheel_steering_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,front_left_wheel_spinning_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,front_right_wheel_spinning_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,rear_left_wheel_spinning_joint_param_name,joint_mappings);
//  insert_parameter_to_map(node,parameters_ns,rear_right_wheel_spinning_joint_param_name,joint_mappings);
//  return joint_mappings;
//}

}
