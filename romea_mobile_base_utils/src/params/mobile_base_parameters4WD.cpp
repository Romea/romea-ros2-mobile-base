#include "romea_mobile_base_utils/params/mobile_base_parameters4WD.hpp"
#include "romea_mobile_base_utils/params/mobile_base_control_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_inertia_parameters.hpp"
#include <romea_common_utils/params/eigen_parameters.hpp>

namespace  {
const std::string geometry_param_name = "geometry";
const std::string wheels_speed_control_param_name = "wheels_speed_control";
const std::string control_point_param_name="control_point";
const std::string inertia_param_name="inertia";
}

namespace romea {

void declare_mobile_base_info_4WD(std::shared_ptr<rclcpp::Node> node,
                                  const std::string & parameters_ns)
{
  declare_two_wheeled_axles_info(
        node,full_param_name(parameters_ns,geometry_param_name));

  declare_wheel_speed_control_info(
        node,full_param_name(parameters_ns,wheels_speed_control_param_name));

  declare_inertia_info(
        node,full_param_name(parameters_ns,inertia_param_name));

  declare_eigen_vector_parameter<Eigen::Vector3d>(
        node,parameters_ns,control_point_param_name);
}

void get_mobile_base_info_4WD(std::shared_ptr<rclcpp::Node> node,
                              const std::string & parameters_ns,
                              MobileBaseInfo4WD & mobile_base_info)
{
  mobile_base_info.geometry= get_two_wheeled_axles_info(
        node,full_param_name(parameters_ns,geometry_param_name));

  mobile_base_info.wheelsSpeedControl = get_wheel_speed_control_info(
        node,full_param_name(parameters_ns,wheels_speed_control_param_name));

  mobile_base_info.inertia = get_inertia_info(
        node,full_param_name(parameters_ns,inertia_param_name));

  mobile_base_info.controlPoint = get_eigen_vector_parameter<Eigen::Vector3d>(
        node,parameters_ns,control_point_param_name);

}

}

