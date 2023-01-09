// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_common_utils/params/node_parameters.hpp>

// std
#include <memory>
#include <string>
#include <vector>

// local
#include "romea_mobile_base_controllers/interfaces/controller_interface2TD.hpp"

namespace
{
const char left_sprocket_wheel_spinning_joint_param_name[] =
  "left_sprocket_wheel_spinning_joint_name";
const char right_sprocket_wheel_spinning_joint_param_name[] =
  "right_sprocket_wheel_spinning_joint_name";
}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2TD::ControllerInterface2TD(const MobileBaseInfo2TD & mobile_base_info)
: virtual_tracks_radius_(mobile_base_info.geometry.tracks.sprocketWheel.radius +
    mobile_base_info.geometry.tracks.thickness)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::write(
  const OdometryFrame2TD & command,
  LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID].
  set_value(command.leftTrackLinearSpeed / virtual_tracks_radius_);
  loaned_command_interfaces[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID].
  set_value(command.rightTrackLinearSpeed / virtual_tracks_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::read(
  const LoanedStateInterfaces & loaned_state_interfaces,
  OdometryFrame2TD & measurement) const
{
  measurement.leftTrackLinearSpeed = virtual_tracks_radius_ *
    loaned_state_interfaces[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rightTrackLinearSpeed = virtual_tracks_radius_ *
    loaned_state_interfaces[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID].get_value();
}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::declare_joints_names(
  std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(
    node, parameters_ns,
    left_sprocket_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(
    node, parameters_ns,
    right_sprocket_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2TD::get_joints_names(
  std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(
      node, parameters_ns,
      left_sprocket_wheel_spinning_joint_param_name),
    get_parameter<std::string>(
      node, parameters_ns,
      right_sprocket_wheel_spinning_joint_param_name)};
}


//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2TD::hardware_interface_names(
  const std::vector<std::string> & joints_names)
{
  return {hardware_velocity_interface_name(joints_names[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID])};
}

}  // namespace romea
