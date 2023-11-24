// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <memory>
#include <string>
#include <vector>

// romea
#include "romea_common_utils/params/node_parameters.hpp"

// local
#include "romea_mobile_base_controllers/interfaces/controller_interface1FAS2RWD.hpp"

namespace
{
const char front_axle_steering_joint_param_name[] = "front_axle_steering_joint_name";
const char rear_left_wheel_spinning_joint_param_name[] = "rear_left_wheel_spinning_joint_name";
const char rear_right_wheel_spinning_joint_param_name[] = "rear_right_wheel_spinning_joint_name";
}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
ControllerInterface1FAS2RWD::ControllerInterface1FAS2RWD(
  const core::MobileBaseInfo1FAS2RWD & mobile_base_info)
: rear_wheels_radius_(mobile_base_info.geometry.rearAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::write(
  const core::OdometryFrame1FAS2RWD & command,
  LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[FRONT_AXLE_STEERING_JOINT_ID].
  set_value(command.frontAxleSteeringAngle);
  loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].
  set_value(command.rearLeftWheelLinearSpeed / rear_wheels_radius_);
  loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].
  set_value(command.rearRightWheelLinearSpeed / rear_wheels_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::read(
  const LoanedStateInterfaces & loaned_state_interfaces,
  core::OdometryFrame1FAS2RWD & measurement)const
{
  measurement.frontAxleSteeringAngle =
    loaned_state_interfaces[FRONT_AXLE_STEERING_JOINT_ID].get_value();
  measurement.rearLeftWheelLinearSpeed = rear_wheels_radius_ *
    loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rearRightWheelLinearSpeed = rear_wheels_radius_ *
    loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::declare_joints_names(
  std::shared_ptr<HardwareInterfaceNode> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node, parameters_ns, front_axle_steering_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, rear_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, rear_right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2RWD::get_joints_names(
  std::shared_ptr<HardwareInterfaceNode> node, const std::string & parameters_ns)
{
  return{get_parameter<std::string>(node, parameters_ns, front_axle_steering_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, rear_left_wheel_spinning_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, rear_right_wheel_spinning_joint_param_name)};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2RWD::hardware_interface_names(
  const std::vector<std::string> & joints_names)
{
  return {hardware_position_interface_name(joints_names[FRONT_AXLE_STEERING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}  // namespace ros2
}  // namespace romea
