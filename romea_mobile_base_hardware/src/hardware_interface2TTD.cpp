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
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl2TTD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2TTD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HardwareInterface2TTD::Configuration::Configuration(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & parameters_prefix)
{
  left_sprocket_wheel_spinning_joint_info =
    get_joint_info(hardware_info, parameters_prefix, "left_sprocket_wheel_spinning_joint_name");
  right_sprocket_wheel_spinning_joint_info =
    get_joint_info(hardware_info, parameters_prefix, "right_sprocket_wheel_spinning_joint_name");
  left_idler_wheel_spinning_joint_info =
    get_joint_info(hardware_info, parameters_prefix, "left_idler_wheel_spinning_joint_name");
  right_idler_wheel_spinning_joint_info =
    get_joint_info(hardware_info, parameters_prefix, "right_idler_wheel_spinning_joint_name");
  front_left_roller_wheel_spinning_joint_info =
    get_joint_info(hardware_info, parameters_prefix, "front_left_roller_wheel_spinning_joint_name");
  front_right_roller_wheel_spinning_joint_info =
    get_joint_info(
    hardware_info, parameters_prefix, "front_right_roller_wheel_spinning_joint_name");
  rear_left_roller_wheel_spinning_joint_info =
    get_joint_info(hardware_info, parameters_prefix, "rear_left_roller_wheel_spinning_joint_name");
  rear_right_roller_wheel_spinning_joint_info =
    get_joint_info(hardware_info, parameters_prefix, "rear_right_roller_wheel_spinning_joint_name");
  command_interface_type =
    get_parameter_or<std::string>(
      hardware_info,
      parameters_prefix,
      "command_interface_type",
      hardware_interface::HW_IF_VELOCITY);
  idler_wheel_radius =
    get_parameter<double>(hardware_info, parameters_prefix, "idler_wheel_radius");
  roller_wheel_radius =
    get_parameter<double>(hardware_info, parameters_prefix, "roller_wheel_radius");
  sprocket_wheel_radius =
    get_parameter<double>(hardware_info, parameters_prefix, "sprocket_wheel_radius");
  track_thickness =
    get_parameter<double>(hardware_info, parameters_prefix, "track_thickness");
}

//-----------------------------------------------------------------------------
HardwareInterface2TTD::HardwareInterface2TTD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_sprocket_wheel_spinning_joint_(
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo2TTD::get_left_sprocket_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  right_sprocket_wheel_spinning_joint_(
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo2TTD::get_right_sprocket_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  left_idler_wheel_spinning_joint_feedback_(
    HardwareInfo2TTD::get_left_idler_wheel_spinning_joint_info(hardware_info)),
  right_idler_wheel_spinning_joint_feedback_(
    HardwareInfo2TTD::get_right_idler_wheel_spinning_joint_info(hardware_info)),
  front_left_roller_wheel_spinning_joint_feedback_(
    HardwareInfo2TTD::get_front_left_roller_wheel_spinning_joint_info(hardware_info)),
  front_right_roller_wheel_spinning_joint_feedback_(
    HardwareInfo2TTD::get_front_right_roller_wheel_spinning_joint_info(hardware_info)),
  rear_left_roller_wheel_spinning_joint_feedback_(
    HardwareInfo2TTD::get_rear_left_roller_wheel_spinning_joint_info(hardware_info)),
  rear_right_roller_wheel_spinning_joint_feedback_(
    HardwareInfo2TTD::get_rear_right_roller_wheel_spinning_joint_info(hardware_info)),
  idler_wheel_radius_(get_idler_wheel_radius(hardware_info)),
  roller_wheel_radius_(get_roller_wheel_radius(hardware_info)),
  sprocket_wheel_radius_(get_sprocket_wheel_radius(hardware_info)),
  track_thickness_(get_track_thickness(hardware_info))
{
}

//-----------------------------------------------------------------------------
HardwareInterface2TTD::HardwareInterface2TTD(const Configuration & configuration)
: left_sprocket_wheel_spinning_joint_(
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    configuration.left_sprocket_wheel_spinning_joint_info,
    configuration.command_interface_type),
  right_sprocket_wheel_spinning_joint_(
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    configuration.right_sprocket_wheel_spinning_joint_info,
    configuration.command_interface_type),
  left_idler_wheel_spinning_joint_feedback_(configuration.left_idler_wheel_spinning_joint_info),
  right_idler_wheel_spinning_joint_feedback_(configuration.right_idler_wheel_spinning_joint_info),
  front_left_roller_wheel_spinning_joint_feedback_(
    configuration.front_left_roller_wheel_spinning_joint_info),
  front_right_roller_wheel_spinning_joint_feedback_(
    configuration.front_right_roller_wheel_spinning_joint_info),
  rear_left_roller_wheel_spinning_joint_feedback_(
    configuration.rear_left_roller_wheel_spinning_joint_info),
  rear_right_roller_wheel_spinning_joint_feedback_(
    configuration.rear_right_roller_wheel_spinning_joint_info),
  idler_wheel_radius_(configuration.idler_wheel_radius),
  roller_wheel_radius_(configuration.roller_wheel_radius),
  sprocket_wheel_radius_(configuration.sprocket_wheel_radius),
  track_thickness_(configuration.track_thickness)
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> HardwareInterface2TTD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  left_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  right_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  front_left_roller_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  front_right_roller_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_left_roller_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_right_roller_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface> HardwareInterface2TTD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
core::HardwareCommand2TD HardwareInterface2TTD::get_hardware_command() const
{
  // *INDENT-OFF*
  return {
    left_sprocket_wheel_spinning_joint_.get_command(),
    right_sprocket_wheel_spinning_joint_.get_command()};
  // *INDENT-ON*
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState HardwareInterface2TTD::get_joint_state_command()
{
  auto joint_states = make_joint_state_msg(2);
  left_sprocket_wheel_spinning_joint_.write_command(joint_states);
  right_sprocket_wheel_spinning_joint_.write_command(joint_states);
  return joint_states;
}

//-----------------------------------------------------------------------------
void HardwareInterface2TTD::set_feedback(const core::HardwareState2TD & hardware_state)
{
  left_sprocket_wheel_spinning_joint_.set_feedback(hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.rightSprocketWheelSpinningMotion);

  // complete_feedback_(hardware_state);
}

//-----------------------------------------------------------------------------
void HardwareInterface2TTD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  left_sprocket_wheel_spinning_joint_.read_feedback(joint_states);
  right_sprocket_wheel_spinning_joint_.read_feedback(joint_states);

  // core::HardwareState2TD hardware_state = {
  //   left_sprocket_wheel_spinning_joint_.get_feedback(),
  //   right_sprocket_wheel_spinning_joint_.get_feedback(),
  // };

  // complete_feedback_(hardware_state);
}

//-----------------------------------------------------------------------------
void HardwareInterface2TTD::complete_feedback_(const core::HardwareState2TD & hardware_state)
{
  core::SimulationState2TTD simulation_state = toSimulationState2TTD(
    sprocket_wheel_radius_,
    idler_wheel_radius_,
    roller_wheel_radius_,
    track_thickness_,
    hardware_state);

  left_idler_wheel_spinning_joint_feedback_.set(simulation_state.leftIdlerWheelSpinningMotion);
  right_idler_wheel_spinning_joint_feedback_.set(simulation_state.rightIdlerWheelSpinningMotion);
  front_left_roller_wheel_spinning_joint_feedback_.set(
    simulation_state.frontLeftRollerWheelSpinningMotion);
  front_right_roller_wheel_spinning_joint_feedback_.set(
    simulation_state.frontRightRollerWheelSpinningMotion);
  rear_left_roller_wheel_spinning_joint_feedback_.set(
    simulation_state.rearLeftRollerWheelSpinningMotion);
  rear_right_roller_wheel_spinning_joint_feedback_.set(
    simulation_state.rearRightRollerWheelSpinningMotion);
}

}  // namespace ros2
}  // namespace romea
