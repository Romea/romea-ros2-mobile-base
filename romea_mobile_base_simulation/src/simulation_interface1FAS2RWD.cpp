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
#include "romea_mobile_base_hardware/hardware_info.hpp"

// local
#include "romea_mobile_base_simulation/simulation_interface1FAS2RWD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface1FAS2RWD::SimulationInterface1FAS2RWD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
:  front_axle_steering_joint_(
    FRONT_AXLE_STEERING_JOINT_ID,
    hardware_info.joints[FRONT_AXLE_STEERING_JOINT_ID]),
  rear_left_wheel_spinning_joint_(
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint_(
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  front_left_wheel_steering_joint_(
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID,
    hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint_(
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID,
    hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint_(
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  front_right_wheel_spinning_joint_(
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  wheelbase_(get_parameter<double>(hardware_info, "wheelbase")),
  front_track_(get_parameter<double>(hardware_info, "front_track")),
  front_wheel_radius_(get_parameter<double>(hardware_info, "front_wheel_radius")),
  front_hub_carrier_offset_(get_parameter<double>(hardware_info, "front_hub_carrier_offset")),
  rear_wheel_radius_(get_parameter<double>(hardware_info, "rear_wheel_radius"))
{
}

//-----------------------------------------------------------------------------
core::SimulationCommand1FAS2RWD SimulationInterface1FAS2RWD::get_hardware_command()
{
  core::HardwareCommand1FAS2RWD command = {
    front_axle_steering_joint_.get_command(),
    rear_left_wheel_spinning_joint_.get_command(),
    rear_right_wheel_spinning_joint_.get_command()};

  return toSimulationCommand1FAS2RWD(
    wheelbase_,
    front_track_,
    front_hub_carrier_offset_,
    front_wheel_radius_,
    rear_wheel_radius_,
    command);
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState SimulationInterface1FAS2RWD::get_joint_state_command()
{
  auto hardware_command = get_hardware_command();

  front_left_wheel_steering_joint_.set_command(
    hardware_command.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_command(
    hardware_command.frontRightWheelSteeringAngle);
  front_left_wheel_spinning_joint_.set_command(
    hardware_command.frontLeftWheelSpinningSetPoint);
  front_right_wheel_spinning_joint_.set_command(
    hardware_command.frontRightWheelSpinningSetPoint);

  auto joint_state_command = make_joint_state_msg(7);
  front_axle_steering_joint_.write_command(joint_state_command);
  front_left_wheel_spinning_joint_.write_command(joint_state_command);
  front_right_wheel_spinning_joint_.write_command(joint_state_command);
  rear_left_wheel_spinning_joint_.write_command(joint_state_command);
  rear_right_wheel_spinning_joint_.write_command(joint_state_command);
  front_left_wheel_steering_joint_.write_command(joint_state_command);
  front_right_wheel_steering_joint_.write_command(joint_state_command);

  return joint_state_command;
}

//-----------------------------------------------------------------------------
void SimulationInterface1FAS2RWD::set_feedback(
  const core::SimulationState1FAS2RWD & simulation_state)
{
  auto hardware_state = toHardwareState1FAS2RWD(
    wheelbase_,
    front_track_,
    simulation_state);

  front_axle_steering_joint_.set_feedback(
    hardware_state.frontAxleSteeringAngle);
  rear_left_wheel_spinning_joint_.set_feedback(
    hardware_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.set_feedback(
    hardware_state.rearRightWheelSpinningMotion);

  front_left_wheel_spinning_joint_.set_feedback(
    simulation_state.frontLeftWheelSpinningMotion);
  front_right_wheel_spinning_joint_.set_feedback(
    simulation_state.frontRightWheelSpinningMotion);
  front_left_wheel_steering_joint_.set_feedback(
    simulation_state.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_feedback(
    simulation_state.frontRightWheelSteeringAngle);
}

//-----------------------------------------------------------------------------
void SimulationInterface1FAS2RWD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  front_axle_steering_joint_.read_feedback(joint_states);
  front_left_wheel_steering_joint_.read_feedback(joint_states);
  front_right_wheel_steering_joint_.read_feedback(joint_states);
  rear_left_wheel_spinning_joint_.read_feedback(joint_states);
  rear_right_wheel_spinning_joint_.read_feedback(joint_states);
  front_left_wheel_spinning_joint_.read_feedback(joint_states);
  front_right_wheel_spinning_joint_.read_feedback(joint_states);

  core::SimulationState1FAS2RWD simulation_state = {
    front_axle_steering_joint_.get_feedback(),
    front_left_wheel_steering_joint_.get_feedback(),
    front_right_wheel_steering_joint_.get_feedback(),
    rear_left_wheel_spinning_joint_.get_feedback(),
    rear_right_wheel_spinning_joint_.get_feedback(),
    front_left_wheel_spinning_joint_.get_feedback(),
    front_right_wheel_spinning_joint_.get_feedback()};

  auto hardware_state = toHardwareState1FAS2RWD(
    wheelbase_,
    front_track_,
    simulation_state);

  front_axle_steering_joint_.set_feedback(
    hardware_state.frontAxleSteeringAngle);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface1FAS2RWD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_axle_steering_joint_.export_state_interface(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_left_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_right_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface1FAS2RWD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_axle_steering_joint_.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

}  // namespace ros2
}  // namespace romea
