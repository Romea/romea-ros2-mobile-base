// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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
#include "romea_common_utils/joint_states.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWC2RWD.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface2FWC2RWD::SimulationInterface2FWC2RWD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: SimulationInterface2FWC2RWD(
    [&]() {
      Configuration configuration;
      configuration.front_left_wheel_swiveling_joint_info =
        hardware_info.joints[FRONT_LEFT_WHEEL_SWIVELING_JOINT_INFO_ID];
      configuration.front_right_wheel_swiveling_joint_info =
        hardware_info.joints[FRONT_RIGHT_WHEEL_SWIVELING_JOINT_INFO_ID];
      configuration.front_left_wheel_spinning_joint_info =
        hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_INFO_ID];
      configuration.front_right_wheel_spinning_joint_info =
        hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_INFO_ID];
      configuration.rear_left_wheel_spinning_joint_info =
        hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_INFO_ID];
      configuration.rear_right_wheel_spinning_joint_info =
        hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_INFO_ID];
      configuration.spinning_joint_command_interface_type = spinning_joint_command_interface_type;
      configuration.wheelbase = get_wheelbase(hardware_info);
      configuration.front_track = get_front_track(hardware_info);
      configuration.front_wheel_x_offset =
        get_parameter<double>(hardware_info, "front_wheel_x_offset");
      configuration.front_wheel_radius = get_front_wheel_radius(hardware_info);
      configuration.rear_wheel_radius = get_rear_wheel_radius(hardware_info);
      return configuration;
    }())
{
}

//-----------------------------------------------------------------------------
SimulationInterface2FWC2RWD::Configuration::Configuration(
  const hardware_interface::HardwareInfo & hardware_info, const std::string & parameters_prefix)
: front_left_wheel_swiveling_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_left_wheel_swiveling_joint_name")),
  front_right_wheel_swiveling_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_right_wheel_swiveling_joint_name")),
  front_left_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_left_wheel_spinning_joint_name")),
  front_right_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_right_wheel_spinning_joint_name")),
  rear_left_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "rear_left_wheel_spinning_joint_name")),
  rear_right_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "rear_right_wheel_spinning_joint_name")),
  spinning_joint_command_interface_type(
    get_parameter_or<std::string>(
      hardware_info, parameters_prefix, "spinning_joint_command_interface_type", "velocity")),
  wheelbase(get_parameter<double>(hardware_info, parameters_prefix, "wheelbase")),
  front_track(get_parameter<double>(hardware_info, parameters_prefix, "front_track")),
  front_wheel_x_offset(
    get_parameter<double>(hardware_info, parameters_prefix, "front_wheel_x_offset")),
  front_wheel_radius(
    get_parameter<double>(hardware_info, parameters_prefix, "front_wheel_radius")),
  rear_wheel_radius(get_parameter<double>(hardware_info, parameters_prefix, "rear_wheel_radius"))
{
}

//-----------------------------------------------------------------------------
SimulationInterface2FWC2RWD::SimulationInterface2FWC2RWD(const Configuration & configuration)
: front_left_wheel_swiveling_joint_(configuration.front_left_wheel_swiveling_joint_info),
  front_right_wheel_swiveling_joint_(configuration.front_right_wheel_swiveling_joint_info),
  front_left_wheel_spinning_joint_feedback_(configuration.front_left_wheel_spinning_joint_info),
  front_right_wheel_spinning_joint_feedback_(configuration.front_right_wheel_spinning_joint_info),
  rear_left_wheel_spinning_joint_(
    REAR_LEFT_WHEEL_SPINNING_COMMAND_ID,
    configuration.rear_left_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint_(
    REAR_RIGHT_WHEEL_SPINNING_COMMAND_ID,
    configuration.rear_right_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  wheelbase_(configuration.wheelbase),
  front_track_(configuration.front_track),
  front_wheel_x_offset_(configuration.front_wheel_x_offset),
  front_wheel_radius_(configuration.front_wheel_radius),
  rear_wheel_radius_(configuration.rear_wheel_radius)
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const SimulationInterface2FWC2RWD::Configuration & configuration)
{
  return {
    make_gazebo_joint_info(configuration.front_left_wheel_swiveling_joint_info, ""),
    make_gazebo_joint_info(configuration.front_right_wheel_swiveling_joint_info, ""),
    make_gazebo_joint_info(configuration.front_left_wheel_spinning_joint_info, ""),
    make_gazebo_joint_info(configuration.front_right_wheel_spinning_joint_info, ""),
    make_gazebo_joint_info(
      configuration.rear_left_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.rear_right_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type)};
}

//-----------------------------------------------------------------------------
core::SimulationCommand2FWC2RWD SimulationInterface2FWC2RWD::get_hardware_command()
{
  core::HardwareCommand2FWC2RWD command = {
    rear_left_wheel_spinning_joint_.get_command(), rear_right_wheel_spinning_joint_.get_command()};

  return toSimulationCommand2FWC2RWD(command);
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState SimulationInterface2FWC2RWD::get_joint_state_command()
{
  auto joint_state_command = make_joint_state_msg(2);
  rear_left_wheel_spinning_joint_.write_command(joint_state_command);
  rear_right_wheel_spinning_joint_.write_command(joint_state_command);
  return joint_state_command;
}

//-----------------------------------------------------------------------------
void SimulationInterface2FWC2RWD::set_feedback(
  const core::SimulationState2FWC2RWD & simulation_state)
{
  front_left_wheel_swiveling_joint_.set_feedback(simulation_state.frontLeftWheelSwivelingAngle);
  front_right_wheel_swiveling_joint_.set_feedback(simulation_state.frontRightWheelSwivelingAngle);
  front_left_wheel_spinning_joint_feedback_.set(simulation_state.frontLeftWheelSpinningMotion);
  front_right_wheel_spinning_joint_feedback_.set(simulation_state.frontRightWheelSpinningMotion);
  rear_left_wheel_spinning_joint_.set_feedback(simulation_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.set_feedback(simulation_state.rearRightWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void SimulationInterface2FWC2RWD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  front_left_wheel_swiveling_joint_.read_feedback(joint_states);
  front_right_wheel_swiveling_joint_.read_feedback(joint_states);

  core::RotationalMotionState front_left_wheel_spinning_state;
  auto front_left_wheel_spinning_id = romea::ros2::get_joint_id(
    joint_states, front_left_wheel_spinning_joint_feedback_.position.get_joint_name());
  front_left_wheel_spinning_state.position =
    get_position(joint_states, front_left_wheel_spinning_id);
  front_left_wheel_spinning_state.velocity =
    get_velocity(joint_states, front_left_wheel_spinning_id);
  front_left_wheel_spinning_state.torque = get_effort(joint_states, front_left_wheel_spinning_id);
  front_left_wheel_spinning_joint_feedback_.set(front_left_wheel_spinning_state);

  core::RotationalMotionState front_right_wheel_spinning_state;
  auto front_right_wheel_spinning_id = romea::ros2::get_joint_id(
    joint_states, front_right_wheel_spinning_joint_feedback_.position.get_joint_name());
  front_right_wheel_spinning_state.position =
    get_position(joint_states, front_right_wheel_spinning_id);
  front_right_wheel_spinning_state.velocity =
    get_velocity(joint_states, front_right_wheel_spinning_id);
  front_right_wheel_spinning_state.torque = get_effort(joint_states, front_right_wheel_spinning_id);
  front_right_wheel_spinning_joint_feedback_.set(front_right_wheel_spinning_state);

  rear_left_wheel_spinning_joint_.read_feedback(joint_states);
  rear_right_wheel_spinning_joint_.read_feedback(joint_states);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2FWC2RWD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_left_wheel_swiveling_joint_.export_state_interface(state_interfaces);
  front_right_wheel_swiveling_joint_.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2FWC2RWD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  rear_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

}  // namespace ros2
}  // namespace romea
