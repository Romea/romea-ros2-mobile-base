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
#include "romea_mobile_base_controllers/interfaces/controller_interface2WD.hpp"

namespace
{
const char left_wheel_spinning_joint_param_name[] = "left_wheel_spinning_joint_name";
const char right_wheel_spinning_joint_param_name[] = "right_wheel_spinning_joint_name";
}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
ControllerInterface2WD::ControllerInterface2WD(const core::MobileBaseInfo2WD & mobile_base_info)
: wheels_radius_(mobile_base_info.geometry.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::write(
  const core::OdometryFrame2WD & command,
  LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[LEFT_WHEEL_SPINNING_JOINT_ID].set_value(
    command.leftWheelLinearSpeed / wheels_radius_);
  loaned_command_interfaces[RIGHT_WHEEL_SPINNING_JOINT_ID].set_value(
    command.rightWheelLinearSpeed / wheels_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::read(
  const LoanedStateInterfaces & loaned_state_interfaces,
  core::OdometryFrame2WD & measurement) const
{
  measurement.leftWheelLinearSpeed = wheels_radius_ *
    loaned_state_interfaces[LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rightWheelLinearSpeed = wheels_radius_ *
    loaned_state_interfaces[RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::declare_joints_names(
  std::shared_ptr<HardwareInterfaceNode> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node, parameters_ns, left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node, parameters_ns, right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2WD::get_joints_names(
  std::shared_ptr<HardwareInterfaceNode> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node, parameters_ns, left_wheel_spinning_joint_param_name),
    get_parameter<std::string>(node, parameters_ns, right_wheel_spinning_joint_param_name)};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2WD::hardware_interface_names(
  const std::vector<std::string> & joints_names)
{
  return {hardware_velocity_interface_name(joints_names[LEFT_WHEEL_SPINNING_JOINT_ID]),
    hardware_velocity_interface_name(joints_names[RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}  // namespace ros2
}  // namespace romea
