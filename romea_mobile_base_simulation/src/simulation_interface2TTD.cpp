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

// local
#include "romea_mobile_base_simulation/simulation_interface2TTD.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface2TTD::SimulationInterface2TTD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_sprocket_wheel_spinning_joint_(
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_sprocket_wheel_spinning_joint_(
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  left_idler_wheel_spinning_joint_(
    LEFT_IDLER_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[LEFT_IDLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  right_idler_wheel_spinning_joint_(
    RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  front_left_roller_wheel_spinning_joint_(
    FRONT_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[FRONT_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  front_right_roller_wheel_spinning_joint_(
    FRONT_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[FRONT_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_left_roller_wheel_spinning_joint_(
    REAR_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[REAR_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  rear_right_roller_wheel_spinning_joint_(
    REAR_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[REAR_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID],
    command_interface_type),
  idler_wheel_radius_(get_parameter<double>(hardware_info, "idler_wheel_radius")),
  roller_wheel_radius_(get_parameter<double>(hardware_info, "roller_wheel_radius")),
  sprocket_wheel_radius_(get_parameter<double>(hardware_info, "sprocket_wheel_radius")),
  track_thickness_(get_parameter<double>(hardware_info, "track_thickness"))
{
}

//-----------------------------------------------------------------------------
core::SimulationCommand2TTD SimulationInterface2TTD::get_hardware_command()
{
  core::HardwareCommand2TD command = {
    left_sprocket_wheel_spinning_joint_.get_command(),
    right_sprocket_wheel_spinning_joint_.get_command(),
  };

  return toSimulationCommand2TTD(
    sprocket_wheel_radius_,
    idler_wheel_radius_,
    roller_wheel_radius_,
    track_thickness_,
    command);
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState SimulationInterface2TTD::get_joint_state_command()
{
  auto hardware_command = get_hardware_command();
  left_idler_wheel_spinning_joint_.set_command(
    hardware_command.leftIdlerWheelSpinningSetPoint);
  right_idler_wheel_spinning_joint_.set_command(
    hardware_command.rightIdlerWheelSpinningSetPoint);
  front_left_roller_wheel_spinning_joint_.set_command(
    hardware_command.frontLeftRollerWheelSpinningSetPoint);
  front_right_roller_wheel_spinning_joint_.set_command(
    hardware_command.frontRightRollerWheelSpinningSetPoint);
  rear_left_roller_wheel_spinning_joint_.set_command(
    hardware_command.rearLeftRollerWheelSpinningSetPoint);
  rear_right_roller_wheel_spinning_joint_.set_command(
    hardware_command.rearRightRollerWheelSpinningSetPoint);

  auto joint_state_command = make_joint_state_msg(8);
  left_sprocket_wheel_spinning_joint_.write_command(joint_state_command);
  right_sprocket_wheel_spinning_joint_.write_command(joint_state_command);
  left_idler_wheel_spinning_joint_.write_command(joint_state_command);
  right_idler_wheel_spinning_joint_.write_command(joint_state_command);
  front_left_roller_wheel_spinning_joint_.write_command(joint_state_command);
  front_right_roller_wheel_spinning_joint_.write_command(joint_state_command);
  rear_left_roller_wheel_spinning_joint_.write_command(joint_state_command);
  rear_right_roller_wheel_spinning_joint_.write_command(joint_state_command);

  return joint_state_command;
}

//-----------------------------------------------------------------------------
void SimulationInterface2TTD::set_feedback(const core::SimulationState2TTD & simulation_state)
{
  auto hardware_state = toHardwareState2TTD(
    sprocket_wheel_radius_,
    roller_wheel_radius_,
    track_thickness_,
    simulation_state);

  left_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.rightSprocketWheelSpinningMotion);
  left_idler_wheel_spinning_joint_.set_feedback(
    simulation_state.leftIdlerWheelSpinningMotion);
  right_idler_wheel_spinning_joint_.set_feedback(
    simulation_state.rightIdlerWheelSpinningMotion);
  front_left_roller_wheel_spinning_joint_.set_feedback(
    simulation_state.frontLeftRollerWheelSpinningMotion);
  front_right_roller_wheel_spinning_joint_.set_feedback(
    simulation_state.frontRightRollerWheelSpinningMotion);
  rear_left_roller_wheel_spinning_joint_.set_feedback(
    simulation_state.rearLeftRollerWheelSpinningMotion);
  rear_right_roller_wheel_spinning_joint_.set_feedback(
    simulation_state.rearRightRollerWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void SimulationInterface2TTD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  left_sprocket_wheel_spinning_joint_.read_feedback(joint_states);
  right_sprocket_wheel_spinning_joint_.read_feedback(joint_states);
  left_idler_wheel_spinning_joint_.read_feedback(joint_states);
  right_idler_wheel_spinning_joint_.read_feedback(joint_states);
  front_left_roller_wheel_spinning_joint_.read_feedback(joint_states);
  front_right_roller_wheel_spinning_joint_.read_feedback(joint_states);
  rear_left_roller_wheel_spinning_joint_.read_feedback(joint_states);
  rear_right_roller_wheel_spinning_joint_.read_feedback(joint_states);


  core::SimulationState2TTD simulation_state = {
    left_sprocket_wheel_spinning_joint_.get_feedback(),
    right_sprocket_wheel_spinning_joint_.get_feedback(),
    left_idler_wheel_spinning_joint_.get_feedback(),
    right_idler_wheel_spinning_joint_.get_feedback(),
    front_left_roller_wheel_spinning_joint_.get_feedback(),
    front_right_roller_wheel_spinning_joint_.get_feedback(),
    rear_left_roller_wheel_spinning_joint_.get_feedback(),
    rear_right_roller_wheel_spinning_joint_.get_feedback()};

  auto hardware_state = toHardwareState2TTD(
    sprocket_wheel_radius_,
    idler_wheel_radius_,
    track_thickness_,
    simulation_state);

  left_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.rightSprocketWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2TTD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  left_idler_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_idler_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_left_roller_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_roller_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_roller_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_roller_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2TTD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

}  // namespace ros2
}  // namespace romea
