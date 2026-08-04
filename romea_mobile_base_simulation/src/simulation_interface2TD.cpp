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

// local
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_simulation/simulation_interface2TD.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2TD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface2TD::SimulationInterface2TD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: SimulationInterface2TD([&]() {
    Configuration configuration;
    configuration.left_sprocket_wheel_spinning_joint_info =
      hardware_info.joints[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID];
    configuration.right_sprocket_wheel_spinning_joint_info =
      hardware_info.joints[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID];
    configuration.left_idler_wheel_spinning_joint_info =
      hardware_info.joints[LEFT_IDLER_WHEEL_SPINNING_JOINT_ID];
    configuration.right_idler_wheel_spinning_joint_info =
      hardware_info.joints[RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID];
    configuration.spinning_joint_command_interface_type = command_interface_type;
    configuration.sprocket_wheel_radius = get_sprocket_wheel_radius(hardware_info);
    configuration.idler_wheel_radius = get_idler_wheel_radius(hardware_info);
    configuration.track_thickness = get_track_thickness(hardware_info);
    return configuration;
  }())
{
}

//-----------------------------------------------------------------------------
SimulationInterface2TD::Configuration::Configuration(
  const hardware_interface::HardwareInfo & hardware_info, const std::string & parameters_prefix)
: left_sprocket_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "left_sprocket_wheel_spinning_joint_name")),
  right_sprocket_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "right_sprocket_wheel_spinning_joint_name")),
  left_idler_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "left_idler_wheel_spinning_joint_name")),
  right_idler_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "right_idler_wheel_spinning_joint_name")),
  spinning_joint_command_interface_type(
    get_parameter_or<std::string>(
      hardware_info, parameters_prefix, "spinning_joint_command_interface_type", "velocity")),
  sprocket_wheel_radius(
    get_parameter<double>(hardware_info, parameters_prefix, "sprocket_wheel_radius")),
  idler_wheel_radius(
    get_parameter<double>(hardware_info, parameters_prefix, "idler_wheel_radius")),
  track_thickness(get_parameter<double>(hardware_info, parameters_prefix, "track_thickness"))
{
}

//-----------------------------------------------------------------------------
SimulationInterface2TD::SimulationInterface2TD(const Configuration & configuration)
: left_sprocket_wheel_spinning_joint_(
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    configuration.left_sprocket_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  right_sprocket_wheel_spinning_joint_(
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    configuration.right_sprocket_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  left_idler_wheel_spinning_joint_(
    LEFT_IDLER_WHEEL_SPINNING_JOINT_ID,
    configuration.left_idler_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  right_idler_wheel_spinning_joint_(
    RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID,
    configuration.right_idler_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  sprocket_wheel_radius_(configuration.sprocket_wheel_radius),
  idler_wheel_radius_(configuration.idler_wheel_radius),
  track_thickness_(configuration.track_thickness)
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const SimulationInterface2TD::Configuration & configuration)
{
  return {
    make_gazebo_joint_info(
      configuration.left_sprocket_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.right_sprocket_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.left_idler_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.right_idler_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type)};
}

//-----------------------------------------------------------------------------
core::SimulationCommand2TD SimulationInterface2TD::get_hardware_command()
{
  core::HardwareCommand2TD command = {
    left_sprocket_wheel_spinning_joint_.get_command(),
    right_sprocket_wheel_spinning_joint_.get_command(),
  };

  return toSimulationCommand2TD(
    sprocket_wheel_radius_, idler_wheel_radius_, track_thickness_, command);
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState SimulationInterface2TD::get_joint_state_command()
{
  auto hardware_command = get_hardware_command();
  left_idler_wheel_spinning_joint_.set_command(hardware_command.leftIdlerWheelSpinningSetPoint);
  right_idler_wheel_spinning_joint_.set_command(hardware_command.rightIdlerWheelSpinningSetPoint);

  auto joint_state_command = make_joint_state_msg(4);
  left_sprocket_wheel_spinning_joint_.write_command(joint_state_command);
  right_sprocket_wheel_spinning_joint_.write_command(joint_state_command);
  left_idler_wheel_spinning_joint_.write_command(joint_state_command);
  right_idler_wheel_spinning_joint_.write_command(joint_state_command);

  return joint_state_command;
}

//-----------------------------------------------------------------------------
void SimulationInterface2TD::set_feedback(const core::SimulationState2TD & simulation_state)
{
  auto hardware_state = toHardwareState2TD(
    sprocket_wheel_radius_, idler_wheel_radius_, track_thickness_, simulation_state);

  left_sprocket_wheel_spinning_joint_.set_feedback(hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.rightSprocketWheelSpinningMotion);
  left_idler_wheel_spinning_joint_.set_feedback(simulation_state.leftIdlerWheelSpinningMotion);
  right_idler_wheel_spinning_joint_.set_feedback(simulation_state.rightIdlerWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void SimulationInterface2TD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  left_sprocket_wheel_spinning_joint_.read_feedback(joint_states);
  right_sprocket_wheel_spinning_joint_.read_feedback(joint_states);
  left_idler_wheel_spinning_joint_.read_feedback(joint_states);
  right_idler_wheel_spinning_joint_.read_feedback(joint_states);

  core::SimulationState2TD simulation_state = {
    left_sprocket_wheel_spinning_joint_.get_feedback(),
    right_sprocket_wheel_spinning_joint_.get_feedback(),
    left_idler_wheel_spinning_joint_.get_feedback(),
    right_idler_wheel_spinning_joint_.get_feedback()};

  auto hardware_state = toHardwareState2TD(
    sprocket_wheel_radius_, idler_wheel_radius_, track_thickness_, simulation_state);

  left_sprocket_wheel_spinning_joint_.set_feedback(hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.rightSprocketWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> SimulationInterface2TD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  left_idler_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_idler_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2TD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

}  // namespace ros2
}  // namespace romea
