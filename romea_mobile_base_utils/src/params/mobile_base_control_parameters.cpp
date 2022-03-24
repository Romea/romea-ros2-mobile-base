#include "romea_mobile_base_utils/params/mobile_base_control_parameters.hpp"
#include <romea_common_utils/params/node_parameters.hpp>


namespace  {
const std::string sensor_angle_std_param_name = "sensor.angle_std";
const std::string sensor_angle_range_param_name = "sensor.angle_range";
const std::string command_maximal_angle_param_name = "command.maximal_angle";
const std::string command_maximal_angular_speed_param_name = "command.maximal_angular_speed";

const std::string sensor_speed_std_param_name = "sensor.speed_std";
const std::string sensor_speed_range_param_name = "sensor.speed_range";
const std::string command_maximal_speed_param_name = "command.maximal_speed";
const std::string command_maximal_acceleration_param_name = "command.maximal_acceleration";

}

namespace romea {

//-----------------------------------------------------------------------------
void declare_steering_angle_control_info(std::shared_ptr<rclcpp::Node> node,
                                         const std::string & parameters_ns)
{

  declare_parameter<double>(node,parameters_ns,sensor_angle_std_param_name);

  declare_parameter_with_default<double>(node,parameters_ns,
                                         sensor_angle_range_param_name,
                                         std::numeric_limits<double>::max());

  declare_parameter<double>(node,parameters_ns,command_maximal_angle_param_name);
  declare_parameter<double>(node,parameters_ns,command_maximal_angular_speed_param_name);
}

//-----------------------------------------------------------------------------
SteeringAngleControl get_steering_angle_control_info(std::shared_ptr<rclcpp::Node> node,
                                                     const std::string & parameters_ns)
{
  return {{get_parameter<double>(node,parameters_ns,sensor_angle_std_param_name),
        get_parameter<double>(node,parameters_ns,sensor_angle_range_param_name)},
        {get_parameter<double>(node,parameters_ns,command_maximal_angle_param_name),
        get_parameter<double>(node,parameters_ns,command_maximal_angular_speed_param_name)}};

}

//-----------------------------------------------------------------------------
void declare_wheel_speed_control_info(std::shared_ptr<rclcpp::Node> node,
                                      const std::string & parameters_ns)
{

  declare_parameter<double>(node,parameters_ns,sensor_speed_std_param_name);

  declare_parameter_with_default<double>(node,parameters_ns,
                                         sensor_speed_range_param_name,
                                         std::numeric_limits<double>::max());

  declare_parameter<double>(node,parameters_ns,command_maximal_speed_param_name);
  declare_parameter<double>(node, parameters_ns,command_maximal_acceleration_param_name);

}

//-----------------------------------------------------------------------------
WheelSpeedControl get_wheel_speed_control_info(std::shared_ptr<rclcpp::Node> node,
                                               const std::string & parameters_ns)
{
  return {{get_parameter<double>(node,parameters_ns,sensor_speed_std_param_name),
        get_parameter<double>(node,parameters_ns,sensor_speed_range_param_name)},
        {get_parameter<double>(node,parameters_ns,command_maximal_speed_param_name),
        get_parameter<double>(node,parameters_ns,command_maximal_acceleration_param_name)}};

}


}

