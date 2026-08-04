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
#include "romea_mobile_base_simulation/simulation_interface2THD.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2THD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface2THD::SimulationInterface2THD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: SimulationInterface2THD([&]() {
    Configuration configuration;
    configuration.left_sprocket_wheel_spinning_joint_info =
      hardware_info.joints[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID];
    configuration.right_sprocket_wheel_spinning_joint_info =
      hardware_info.joints[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID];
    configuration.front_left_idler_wheel_spinning_joint_info =
      hardware_info.joints[FRONT_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID];
    configuration.front_right_idler_wheel_spinning_joint_info =
      hardware_info.joints[FRONT_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID];
    configuration.rear_left_idler_wheel_spinning_joint_info =
      hardware_info.joints[REAR_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID];
    configuration.rear_right_idler_wheel_spinning_joint_info =
      hardware_info.joints[REAR_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID];
    configuration.spinning_joint_command_interface_type = command_interface_type;
    configuration.idler_wheel_radius = get_idler_wheel_radius(hardware_info);
    configuration.sprocket_wheel_radius = get_sprocket_wheel_radius(hardware_info);
    configuration.track_thickness = get_track_thickness(hardware_info);
    return configuration;
  }())
{
}

//-----------------------------------------------------------------------------
SimulationInterface2THD::Configuration::Configuration(
  const hardware_interface::HardwareInfo & hardware_info, const std::string & parameters_prefix)
: left_sprocket_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "left_sprocket_wheel_spinning_joint_name")),
  right_sprocket_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "right_sprocket_wheel_spinning_joint_name")),
  front_left_idler_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_left_idler_wheel_spinning_joint_name")),
  front_right_idler_wheel_spinning_joint_info(
    get_joint_info(
      hardware_info, parameters_prefix, "front_right_idler_wheel_spinning_joint_name")),
  rear_left_idler_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "rear_left_idler_wheel_spinning_joint_name")),
  rear_right_idler_wheel_spinning_joint_info(
    get_joint_info(
      hardware_info, parameters_prefix, "rear_right_idler_wheel_spinning_joint_name")),
  spinning_joint_command_interface_type(
    get_parameter_or<std::string>(
      hardware_info, parameters_prefix, "spinning_joint_command_interface_type", "velocity")),
  idler_wheel_radius(
    get_parameter<double>(hardware_info, parameters_prefix, "idler_wheel_radius")),
  sprocket_wheel_radius(
    get_parameter<double>(hardware_info, parameters_prefix, "sprocket_wheel_radius")),
  track_thickness(get_parameter<double>(hardware_info, parameters_prefix, "track_thickness"))
{
}

//-----------------------------------------------------------------------------
SimulationInterface2THD::SimulationInterface2THD(const Configuration & configuration)
: left_sprocket_wheel_spinning_joint_(
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    configuration.left_sprocket_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  right_sprocket_wheel_spinning_joint_(
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    configuration.right_sprocket_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  front_left_idler_wheel_spinning_joint_(
    FRONT_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID,
    configuration.front_left_idler_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  front_right_idler_wheel_spinning_joint_(
    FRONT_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID,
    configuration.front_right_idler_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  rear_left_idler_wheel_spinning_joint_(
    REAR_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID,
    configuration.rear_left_idler_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  rear_right_idler_wheel_spinning_joint_(
    REAR_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID,
    configuration.rear_right_idler_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  idler_wheel_radius_(configuration.idler_wheel_radius),
  sprocket_wheel_radius_(configuration.sprocket_wheel_radius),
  track_thickness_(configuration.track_thickness)
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const SimulationInterface2THD::Configuration & configuration)
{
  return {
    make_gazebo_joint_info(
      configuration.left_sprocket_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.right_sprocket_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.front_left_idler_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.front_right_idler_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.rear_left_idler_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.rear_right_idler_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type)};
}

//-----------------------------------------------------------------------------
core::SimulationCommand2THD SimulationInterface2THD::get_hardware_command()
{
  core::HardwareCommand2TD command = {
    left_sprocket_wheel_spinning_joint_.get_command(),
    right_sprocket_wheel_spinning_joint_.get_command(),
  };

  return toSimulationCommand2THD(
    sprocket_wheel_radius_, idler_wheel_radius_, track_thickness_, command);
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState SimulationInterface2THD::get_joint_state_command()
{
  auto hardware_command = get_hardware_command();
  front_left_idler_wheel_spinning_joint_.set_command(
    hardware_command.frontLeftIdlerWheelSpinningSetPoint);
  front_right_idler_wheel_spinning_joint_.set_command(
    hardware_command.frontRightIdlerWheelSpinningSetPoint);
  rear_left_idler_wheel_spinning_joint_.set_command(
    hardware_command.rearLeftIdlerWheelSpinningSetPoint);
  rear_right_idler_wheel_spinning_joint_.set_command(
    hardware_command.rearRightIdlerWheelSpinningSetPoint);

  auto joint_state_command = make_joint_state_msg(6);
  left_sprocket_wheel_spinning_joint_.write_command(joint_state_command);
  right_sprocket_wheel_spinning_joint_.write_command(joint_state_command);
  front_left_idler_wheel_spinning_joint_.write_command(joint_state_command);
  front_right_idler_wheel_spinning_joint_.write_command(joint_state_command);
  rear_left_idler_wheel_spinning_joint_.write_command(joint_state_command);
  rear_right_idler_wheel_spinning_joint_.write_command(joint_state_command);

  return joint_state_command;
}

//-----------------------------------------------------------------------------
void SimulationInterface2THD::set_feedback(const core::SimulationState2THD & simulation_state)
{
  auto hardware_state = toHardwareState2TD(
    sprocket_wheel_radius_, idler_wheel_radius_, track_thickness_, simulation_state);

  left_sprocket_wheel_spinning_joint_.set_feedback(hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.rightSprocketWheelSpinningMotion);
  front_left_idler_wheel_spinning_joint_.set_feedback(
    simulation_state.frontLeftIdlerWheelSpinningMotion);
  front_right_idler_wheel_spinning_joint_.set_feedback(
    simulation_state.frontRightIdlerWheelSpinningMotion);
  rear_left_idler_wheel_spinning_joint_.set_feedback(
    simulation_state.rearLeftIdlerWheelSpinningMotion);
  rear_right_idler_wheel_spinning_joint_.set_feedback(
    simulation_state.rearRightIdlerWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void SimulationInterface2THD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  left_sprocket_wheel_spinning_joint_.read_feedback(joint_states);
  right_sprocket_wheel_spinning_joint_.read_feedback(joint_states);
  front_left_idler_wheel_spinning_joint_.read_feedback(joint_states);
  front_right_idler_wheel_spinning_joint_.read_feedback(joint_states);
  rear_left_idler_wheel_spinning_joint_.read_feedback(joint_states);
  rear_right_idler_wheel_spinning_joint_.read_feedback(joint_states);

  core::SimulationState2THD simulation_state = {
    left_sprocket_wheel_spinning_joint_.get_feedback(),
    right_sprocket_wheel_spinning_joint_.get_feedback(),
    front_left_idler_wheel_spinning_joint_.get_feedback(),
    front_right_idler_wheel_spinning_joint_.get_feedback(),
    rear_left_idler_wheel_spinning_joint_.get_feedback(),
    rear_right_idler_wheel_spinning_joint_.get_feedback()};

  auto hardware_state = toHardwareState2TD(
    sprocket_wheel_radius_, idler_wheel_radius_, track_thickness_, simulation_state);

  left_sprocket_wheel_spinning_joint_.set_feedback(hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.rightSprocketWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> SimulationInterface2THD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_left_idler_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_idler_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_idler_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_idler_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2THD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

}  // namespace ros2
}  // namespace romea
