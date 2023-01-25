// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>
#include <vector>

// romea
#include "romea_common_utils/params/node_parameters.hpp"

// local
#include "romea_mobile_base_controllers/interfaces/controller_interface2FWS2FWD.hpp"

namespace
{
const char front_left_wheel_steering_joint_param_name[] = "front_left_wheel_steering_joint_name";
const char front_right_wheel_steering_joint_param_name[] = "front_right_wheel_steering_joint_name";
const char front_left_wheel_spinning_joint_param_name[] = "front_left_wheel_spinning_joint_name";
const char front_right_wheel_spinning_joint_param_name[] = "front_right_wheel_spinning_joint_name";
}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2FWS2FWD::ControllerInterface2FWS2FWD(
  const MobileBaseInfo2FWS2FWD & mobile_base_info)
: front_wheels_radius_(mobile_base_info.geometry.frontAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2FWD::write(
  const OdometryFrame2FWS2FWD & command,
  LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID].
  set_value(command.frontLeftWheelSteeringAngle);
  loaned_command_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID].
  set_value(command.frontRightWheelSteeringAngle);
  loaned_command_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID].
  set_value(command.frontLeftWheelLinearSpeed / front_wheels_radius_);
  loaned_command_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID].
  set_value(command.frontRightWheelLinearSpeed / front_wheels_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2FWD::read(
  const LoanedStateInterfaces & loaned_state_interfaces,
  OdometryFrame2FWS2FWD & measurement)const
{
  measurement.frontLeftWheelSteeringAngle =
    loaned_state_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID].get_value();
  measurement.frontRightWheelSteeringAngle =
    loaned_state_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID].get_value();
  measurement.frontLeftWheelLinearSpeed = front_wheels_radius_ *
    loaned_state_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.frontRightWheelLinearSpeed = front_wheels_radius_ *
    loaned_state_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2FWD::declare_joints_names(
  std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node, parameters_ns, front_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, front_right_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, front_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, front_right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2FWS2FWD::get_joints_names(
  std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(
      node, parameters_ns,
      front_left_wheel_steering_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, front_right_wheel_steering_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, front_left_wheel_spinning_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, front_right_wheel_spinning_joint_param_name)};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2FWS2FWD::hardware_interface_names(
  const std::vector<std::string> & joints_names)
{
  return {hardware_position_interface_name(joints_names[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
    hardware_position_interface_name(joints_names[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}  // namespace romea
