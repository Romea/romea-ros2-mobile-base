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
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2FWSxxx.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWS2FWD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface2FWS2FWD::SimulationInterface2FWS2FWD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: front_left_wheel_steering_joint_(
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
  rear_left_wheel_spinning_joint_(
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint_(
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID,
    hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],
    spinning_joint_command_interface_type),
  wheelbase_(get_wheelbase(hardware_info)),
  front_track_(get_front_track(hardware_info)),
  front_wheel_radius_(get_front_wheel_radius(hardware_info)),
  front_hub_carrier_offset_(get_front_hub_carrier_offset(hardware_info)),
  rear_track_(get_rear_track(hardware_info)),
  rear_wheel_radius_(get_rear_wheel_radius(hardware_info)),
  rear_hub_carrier_offset_(get_rear_hub_carrier_offset(hardware_info))
{
}

//-----------------------------------------------------------------------------
core::SimulationCommand2FWS2FWD SimulationInterface2FWS2FWD::get_hardware_command()
{
  core::HardwareCommand2FWS2FWD command = {
    front_left_wheel_steering_joint_.get_command(),
    front_right_wheel_steering_joint_.get_command(),
    front_left_wheel_spinning_joint_.get_command(),
    front_right_wheel_spinning_joint_.get_command()};

  return toSimulationCommand2FWS2FWD(
    wheelbase_,
    front_track_,
    rear_track_,
    front_wheel_radius_,
    rear_wheel_radius_,
    front_hub_carrier_offset_,
    rear_hub_carrier_offset_,
    command);
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState SimulationInterface2FWS2FWD::get_joint_state_command()
{
  auto hardware_command = get_hardware_command();

  rear_left_wheel_spinning_joint_.set_command(
    hardware_command.rearLeftWheelSpinningSetPoint);
  rear_right_wheel_spinning_joint_.set_command(
    hardware_command.rearRightWheelSpinningSetPoint);

  auto joint_state_command = make_joint_state_msg(6);
  front_left_wheel_steering_joint_.write_command(joint_state_command);
  front_right_wheel_steering_joint_.write_command(joint_state_command);
  front_left_wheel_spinning_joint_.write_command(joint_state_command);
  front_right_wheel_spinning_joint_.write_command(joint_state_command);
  rear_left_wheel_spinning_joint_.write_command(joint_state_command);
  rear_right_wheel_spinning_joint_.write_command(joint_state_command);
  return joint_state_command;
}

//-----------------------------------------------------------------------------
void SimulationInterface2FWS2FWD::set_feedback(
  const core::SimulationState2FWS2FWD & simulation_state)
{
  front_left_wheel_steering_joint_.set_feedback(
    simulation_state.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_feedback(
    simulation_state.frontRightWheelSteeringAngle);
  front_left_wheel_spinning_joint_.set_feedback(
    simulation_state.frontLeftWheelSpinningMotion),
  front_right_wheel_spinning_joint_.set_feedback(
    simulation_state.frontRightWheelSpinningMotion);
  rear_left_wheel_spinning_joint_.set_feedback(
    simulation_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.set_feedback(
    simulation_state.rearRightWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
void SimulationInterface2FWS2FWD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  front_left_wheel_steering_joint_.read_feedback(joint_states);
  front_right_wheel_steering_joint_.read_feedback(joint_states);
  rear_left_wheel_spinning_joint_.read_feedback(joint_states);
  rear_right_wheel_spinning_joint_.read_feedback(joint_states);
  front_left_wheel_spinning_joint_.read_feedback(joint_states);
  front_right_wheel_spinning_joint_.read_feedback(joint_states);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2FWS2FWD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_left_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_right_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2FWS2FWD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_left_wheel_steering_joint_.export_command_interface(command_interfaces);
  front_right_wheel_steering_joint_.export_command_interface(command_interfaces);
  front_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

}  // namespace ros2
}  // namespace romea
