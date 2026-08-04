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
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "romea_mobile_base_simulation/simulation_interface2AS4WD.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2ASxxx.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface2AS4WD::SimulationInterface2AS4WD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: SimulationInterface2AS4WD([&]() {
    Configuration configuration;
    configuration.front_axle_steering_joint_info =
      hardware_info.joints[FRONT_AXLE_STEERING_JOINT_ID];
    configuration.rear_axle_steering_joint_info = hardware_info.joints[REAR_AXLE_STEERING_JOINT_ID];
    configuration.front_left_wheel_spinning_joint_info =
      hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID];
    configuration.front_right_wheel_spinning_joint_info =
      hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID];
    configuration.rear_left_wheel_spinning_joint_info =
      hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID];
    configuration.rear_right_wheel_spinning_joint_info =
      hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID];
    configuration.front_left_wheel_steering_joint_info =
      hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID];
    configuration.front_right_wheel_steering_joint_info =
      hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID];
    configuration.rear_left_wheel_steering_joint_info =
      hardware_info.joints[REAR_LEFT_WHEEL_STEERING_JOINT_ID];
    configuration.rear_right_wheel_steering_joint_info =
      hardware_info.joints[REAR_RIGHT_WHEEL_STEERING_JOINT_ID];
    configuration.spinning_joint_command_interface_type = spinning_joint_command_interface_type;
    configuration.wheelbase = get_wheelbase(hardware_info);
    configuration.front_track = get_front_track(hardware_info);
    configuration.rear_track = get_rear_track(hardware_info);
    return configuration;
  }())
{
}

//-----------------------------------------------------------------------------
SimulationInterface2AS4WD::Configuration::Configuration(
  const hardware_interface::HardwareInfo & hardware_info, const std::string & parameters_prefix)
: front_axle_steering_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_axle_steering_joint_name")),
  rear_axle_steering_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "rear_axle_steering_joint_name")),
  front_left_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_left_wheel_spinning_joint_name")),
  front_right_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_right_wheel_spinning_joint_name")),
  rear_left_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "rear_left_wheel_spinning_joint_name")),
  rear_right_wheel_spinning_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "rear_right_wheel_spinning_joint_name")),
  front_left_wheel_steering_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_left_wheel_steering_joint_name")),
  front_right_wheel_steering_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "front_right_wheel_steering_joint_name")),
  rear_left_wheel_steering_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "rear_left_wheel_steering_joint_name")),
  rear_right_wheel_steering_joint_info(
    get_joint_info(hardware_info, parameters_prefix, "rear_right_wheel_steering_joint_name")),
  spinning_joint_command_interface_type(
    get_parameter_or<std::string>(
      hardware_info, parameters_prefix, "spinning_joint_command_interface_type", "velocity")),
  wheelbase(get_parameter<double>(hardware_info, parameters_prefix, "wheelbase")),
  front_track(get_parameter<double>(hardware_info, parameters_prefix, "front_track")),
  rear_track(get_parameter<double>(hardware_info, parameters_prefix, "rear_track"))
{
}

//-----------------------------------------------------------------------------
SimulationInterface2AS4WD::SimulationInterface2AS4WD(const Configuration & configuration)
: front_axle_steering_joint_(
    FRONT_AXLE_STEERING_JOINT_ID, configuration.front_axle_steering_joint_info),
  rear_axle_steering_joint_(
    REAR_AXLE_STEERING_JOINT_ID, configuration.rear_axle_steering_joint_info),
  front_left_wheel_spinning_joint_(
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    configuration.front_left_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  front_right_wheel_spinning_joint_(
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID,
    configuration.front_right_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  rear_left_wheel_spinning_joint_(
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    configuration.rear_left_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint_(
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID,
    configuration.rear_right_wheel_spinning_joint_info,
    configuration.spinning_joint_command_interface_type),
  front_left_wheel_steering_joint_(
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID, configuration.front_left_wheel_steering_joint_info),
  front_right_wheel_steering_joint_(
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID, configuration.front_right_wheel_steering_joint_info),
  rear_left_wheel_steering_joint_(
    REAR_LEFT_WHEEL_STEERING_JOINT_ID, configuration.rear_left_wheel_steering_joint_info),
  rear_right_wheel_steering_joint_(
    REAR_RIGHT_WHEEL_STEERING_JOINT_ID, configuration.rear_right_wheel_steering_joint_info),
  wheelbase_(configuration.wheelbase),
  front_track_(configuration.front_track),
  rear_track_(configuration.rear_track)
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const SimulationInterface2AS4WD::Configuration & configuration)
{
  return {
    make_gazebo_joint_info(
      configuration.front_axle_steering_joint_info, hardware_interface::HW_IF_POSITION),
    make_gazebo_joint_info(
      configuration.rear_axle_steering_joint_info, hardware_interface::HW_IF_POSITION),
    make_gazebo_joint_info(
      configuration.front_left_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.front_right_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.rear_left_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.rear_right_wheel_spinning_joint_info,
      configuration.spinning_joint_command_interface_type),
    make_gazebo_joint_info(
      configuration.front_left_wheel_steering_joint_info, hardware_interface::HW_IF_POSITION),
    make_gazebo_joint_info(
      configuration.front_right_wheel_steering_joint_info, hardware_interface::HW_IF_POSITION),
    make_gazebo_joint_info(
      configuration.rear_left_wheel_steering_joint_info, hardware_interface::HW_IF_POSITION),
    make_gazebo_joint_info(
      configuration.rear_right_wheel_steering_joint_info, hardware_interface::HW_IF_POSITION)};
}

//-----------------------------------------------------------------------------
core::SimulationCommand2AS4WD SimulationInterface2AS4WD::get_hardware_command()
{
  core::HardwareCommand2AS4WD command = {
    front_axle_steering_joint_.get_command(),
    rear_axle_steering_joint_.get_command(),
    front_left_wheel_spinning_joint_.get_command(),
    front_right_wheel_spinning_joint_.get_command(),
    rear_left_wheel_spinning_joint_.get_command(),
    rear_right_wheel_spinning_joint_.get_command()};

  return toSimulationCommand2AS4WD(wheelbase_, front_track_, rear_track_, command);
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState SimulationInterface2AS4WD::get_joint_state_command()
{
  auto hardware_command = get_hardware_command();

  front_left_wheel_steering_joint_.set_command(hardware_command.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_command(hardware_command.frontRightWheelSteeringAngle);
  rear_left_wheel_steering_joint_.set_command(hardware_command.rearLeftWheelSteeringAngle);
  rear_right_wheel_steering_joint_.set_command(hardware_command.rearRightWheelSteeringAngle);

  auto joint_state_command = make_joint_state_msg(10);
  front_axle_steering_joint_.write_command(joint_state_command);
  rear_axle_steering_joint_.write_command(joint_state_command);
  front_left_wheel_spinning_joint_.write_command(joint_state_command);
  front_right_wheel_spinning_joint_.write_command(joint_state_command);
  rear_left_wheel_spinning_joint_.write_command(joint_state_command);
  rear_right_wheel_spinning_joint_.write_command(joint_state_command);
  front_left_wheel_steering_joint_.write_command(joint_state_command);
  front_right_wheel_steering_joint_.write_command(joint_state_command);
  rear_left_wheel_steering_joint_.write_command(joint_state_command);
  rear_right_wheel_steering_joint_.write_command(joint_state_command);

  return joint_state_command;
}

//-----------------------------------------------------------------------------
void SimulationInterface2AS4WD::set_feedback(const core::SimulationState2AS4WD & simulation_state)
{
  auto hardware_state =
    toHardwareState2AS4WD(wheelbase_, front_track_, rear_track_, simulation_state);

  front_axle_steering_joint_.set_feedback(hardware_state.frontAxleSteeringAngle);
  rear_axle_steering_joint_.set_feedback(hardware_state.rearAxleSteeringAngle);

  front_left_wheel_spinning_joint_.set_feedback(hardware_state.frontLeftWheelSpinningMotion);
  front_right_wheel_spinning_joint_.set_feedback(hardware_state.frontRightWheelSpinningMotion);
  rear_left_wheel_spinning_joint_.set_feedback(hardware_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.set_feedback(hardware_state.rearRightWheelSpinningMotion);

  front_left_wheel_steering_joint_.set_feedback(simulation_state.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_feedback(simulation_state.frontRightWheelSteeringAngle);
  rear_left_wheel_steering_joint_.set_feedback(simulation_state.rearLeftWheelSteeringAngle);
  rear_right_wheel_steering_joint_.set_feedback(simulation_state.rearRightWheelSteeringAngle);
}

//-----------------------------------------------------------------------------
void SimulationInterface2AS4WD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  front_axle_steering_joint_.read_feedback(joint_states);
  rear_axle_steering_joint_.read_feedback(joint_states);
  front_left_wheel_spinning_joint_.read_feedback(joint_states);
  front_right_wheel_spinning_joint_.read_feedback(joint_states);
  rear_left_wheel_spinning_joint_.read_feedback(joint_states);
  rear_right_wheel_spinning_joint_.read_feedback(joint_states);
  front_left_wheel_steering_joint_.read_feedback(joint_states);
  front_right_wheel_steering_joint_.read_feedback(joint_states);
  rear_left_wheel_steering_joint_.read_feedback(joint_states);
  rear_right_wheel_steering_joint_.read_feedback(joint_states);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> SimulationInterface2AS4WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_axle_steering_joint_.export_state_interface(state_interfaces);
  rear_axle_steering_joint_.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_left_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_right_wheel_steering_joint_.export_state_interface(state_interfaces);
  rear_left_wheel_steering_joint_.export_state_interface(state_interfaces);
  rear_right_wheel_steering_joint_.export_state_interface(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2AS4WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_axle_steering_joint_.export_command_interface(command_interfaces);
  rear_axle_steering_joint_.export_command_interface(command_interfaces);
  front_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

}  // namespace ros2
}  // namespace romea
