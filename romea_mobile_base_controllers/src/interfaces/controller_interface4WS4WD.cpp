// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_common_utils/params/node_parameters.hpp>

// std
#include <memory>
#include <string>
#include <vector>

// local
#include "romea_mobile_base_controllers/interfaces/controller_interface4WS4WD.hpp"

namespace
{
const char front_left_wheel_steering_joint_param_name[] = "front_left_wheel_steering_joint_name";
const char front_right_wheel_steering_joint_param_name[] = "front_right_wheel_steering_joint_name";
const char rear_left_wheel_steering_joint_param_name[] = "rear_left_wheel_steering_joint_name";
const char rear_right_wheel_steering_joint_param_name[] = "rear_right_wheel_steering_joint_name";
const char front_left_wheel_spinning_joint_param_name[] = "front_left_wheel_spinning_joint_name";
const char front_right_wheel_spinning_joint_param_name[] = "front_right_wheel_spinning_joint_name";
const char rear_left_wheel_spinning_joint_param_name[] = "rear_left_wheel_spinning_joint_name";
const char rear_right_wheel_spinning_joint_param_name[] = "rear_right_wheel_spinning_joint_name";
}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface4WS4WD::ControllerInterface4WS4WD(const MobileBaseInfo4WS4WD & mobile_base_info)
: front_wheels_radius_(mobile_base_info.geometry.frontAxle.wheels.radius),
  rear_wheels_radius_(mobile_base_info.geometry.frontAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface4WS4WD::write(
  const OdometryFrame4WS4WD & command,
  LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID].
  set_value(command.frontLeftWheelSteeringAngle);
  loaned_command_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID].
  set_value(command.frontRightWheelSteeringAngle);
  loaned_command_interfaces[REAR_LEFT_WHEEL_STEERING_JOINT_ID].
  set_value(command.rearLeftWheelSteeringAngle);
  loaned_command_interfaces[REAR_RIGHT_WHEEL_STEERING_JOINT_ID].
  set_value(command.rearRightWheelSteeringAngle);
  loaned_command_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID].
  set_value(command.frontLeftWheelLinearSpeed / front_wheels_radius_);
  loaned_command_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID].
  set_value(command.frontRightWheelLinearSpeed / front_wheels_radius_);
  loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].
  set_value(command.rearLeftWheelLinearSpeed / rear_wheels_radius_);
  loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].
  set_value(command.rearRightWheelLinearSpeed / rear_wheels_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface4WS4WD::read(
  const LoanedStateInterfaces & loaned_state_interfaces,
  OdometryFrame4WS4WD & measurement)const
{
  measurement.frontLeftWheelSteeringAngle =
    loaned_state_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID].get_value();
  measurement.frontRightWheelSteeringAngle =
    loaned_state_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID].get_value();
  measurement.rearLeftWheelSteeringAngle =
    loaned_state_interfaces[REAR_LEFT_WHEEL_STEERING_JOINT_ID].get_value();
  measurement.rearRightWheelSteeringAngle =
    loaned_state_interfaces[REAR_RIGHT_WHEEL_STEERING_JOINT_ID].get_value();
  measurement.frontLeftWheelLinearSpeed = front_wheels_radius_ *
    loaned_state_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.frontRightWheelLinearSpeed = front_wheels_radius_ *
    loaned_state_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rearLeftWheelLinearSpeed = rear_wheels_radius_ *
    loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rearRightWheelLinearSpeed = rear_wheels_radius_ *
    loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
}

//-----------------------------------------------------------------------------
void ControllerInterface4WS4WD::declare_joints_names(
  std::shared_ptr<HardwareInterfaceNode> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node, parameters_ns, front_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, front_right_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, rear_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, rear_right_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, front_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, front_right_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, rear_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, rear_right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WS4WD::get_joints_names(
  std::shared_ptr<HardwareInterfaceNode> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(
      node, parameters_ns,
      front_left_wheel_steering_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, front_right_wheel_steering_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, rear_left_wheel_steering_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, rear_right_wheel_steering_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, front_left_wheel_spinning_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, front_right_wheel_spinning_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, rear_left_wheel_spinning_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, rear_right_wheel_spinning_joint_param_name)};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WS4WD::hardware_interface_names(
  const std::vector<std::string> & joints_names)
{
  return {hardware_position_interface_name(joints_names[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
    hardware_position_interface_name(joints_names[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
    hardware_position_interface_name(joints_names[REAR_LEFT_WHEEL_STEERING_JOINT_ID]),
    hardware_position_interface_name(joints_names[REAR_RIGHT_WHEEL_STEERING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}  // namespace romea
