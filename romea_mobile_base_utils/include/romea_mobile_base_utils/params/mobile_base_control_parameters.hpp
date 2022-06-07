#ifndef _romea_SteeringAngleControlParamters_hpp_
#define _romea_SteeringAngleControlParamters_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseControl.hpp>

namespace romea {

void declare_steering_angle_control_info(std::shared_ptr<rclcpp::Node> node,
                                         const std::string & parameters_ns);

void declare_wheel_speed_control_info(std::shared_ptr<rclcpp::Node> node,
                                      const std::string & parameters_ns);

SteeringAngleControl get_steering_angle_control_info(std::shared_ptr<rclcpp::Node> node,
                                                     const std::string & parameters_ns);

WheelSpeedControl get_wheel_speed_control_info(std::shared_ptr<rclcpp::Node> node,
                                               const std::string & parameters_ns);

}

#endif
