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
#include "romea_core_mobile_base/simulation/SimulationControl2FWS2FWD.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2FWSxxx.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS2FWD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HardwareInterface2FWS2FWD::HardwareInterface2FWS2FWD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: front_left_wheel_steering_joint_(
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID,
    HardwareInfo2FWSxxx::get_front_left_wheel_steering_joint_info(hardware_info)),
  front_right_wheel_steering_joint_(
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID,
    HardwareInfo2FWSxxx::get_front_right_wheel_steering_joint_info(hardware_info)),
  front_left_wheel_spinning_joint_(
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo2FWSxxx::get_front_left_wheel_spinning_joint_info(hardware_info),
    spinning_joint_command_interface_type),
  front_right_wheel_spinning_joint_(
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo2FWSxxx::get_front_right_wheel_spinning_joint_info(hardware_info),
    spinning_joint_command_interface_type),
  rear_left_wheel_spinning_joint_feedback_(
    HardwareInfo2FWSxxx::get_rear_left_wheel_spinning_joint_info(hardware_info)),
  rear_right_wheel_spinning_joint_feedback_(
    HardwareInfo2FWSxxx::get_rear_right_wheel_spinning_joint_info(hardware_info)),
  wheelbase_(get_wheelbase(hardware_info)),
  front_track_(get_front_track(hardware_info)),
  rear_track_(get_rear_track(hardware_info)),
  front_wheel_radius_(get_front_wheel_radius(hardware_info)),
  rear_wheel_radius_(get_rear_wheel_radius(hardware_info)),
  front_hub_carrier_offset_(get_front_hub_carrier_offset(hardware_info)),
  rear_hub_carrier_offset_(get_rear_hub_carrier_offset(hardware_info))
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> HardwareInterface2FWS2FWD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_left_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_right_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface> HardwareInterface2FWS2FWD::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_left_wheel_steering_joint_.export_command_interface(command_interfaces);
  front_right_wheel_steering_joint_.export_command_interface(command_interfaces);
  front_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
core::HardwareCommand2FWS2FWD HardwareInterface2FWS2FWD::get_hardware_command() const
{
  // *INDENT-OFF*
  return {front_left_wheel_steering_joint_.get_command(),
      front_right_wheel_steering_joint_.get_command(),
      front_left_wheel_spinning_joint_.get_command(),
      front_right_wheel_spinning_joint_.get_command()};
  // *INDENT-ON*
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState HardwareInterface2FWS2FWD::get_joint_state_command() const
{
  auto joint_states = make_joint_state_msg(4);
  front_left_wheel_steering_joint_.write_command(joint_states);
  front_right_wheel_steering_joint_.write_command(joint_states);
  front_left_wheel_spinning_joint_.write_command(joint_states);
  front_right_wheel_spinning_joint_.write_command(joint_states);
  return joint_states;
}

//-----------------------------------------------------------------------------
void HardwareInterface2FWS2FWD::set_feedback(const core::HardwareState2FWS2FWD & hardware_state)
{
  front_left_wheel_steering_joint_.set_feedback(hardware_state.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_feedback(hardware_state.frontRightWheelSteeringAngle);
  front_left_wheel_spinning_joint_.set_feedback(hardware_state.frontLeftWheelSpinningMotion);
  front_right_wheel_spinning_joint_.set_feedback(hardware_state.frontRightWheelSpinningMotion);

  // complete_feedback_(hardware_state);
}

//-----------------------------------------------------------------------------
void HardwareInterface2FWS2FWD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
{
  front_left_wheel_steering_joint_.read_feedback(joint_states);
  front_right_wheel_steering_joint_.read_feedback(joint_states);
  front_left_wheel_spinning_joint_.read_feedback(joint_states);
  front_right_wheel_spinning_joint_.read_feedback(joint_states);

  // core::HardwareState2FWS2FWD hardware_state = {
  //   front_left_wheel_steering_joint_.get_feedback(),
  //   front_right_wheel_steering_joint_.get_feedback(),
  //   front_left_wheel_spinning_joint_.get_feedback(),
  //   front_right_wheel_spinning_joint_.get_feedback()
  // };

  // complete_feedback_(hardware_state);
}

//-----------------------------------------------------------------------------
void HardwareInterface2FWS2FWD::complete_feedback_(
  const core::HardwareState2FWS2FWD & hardware_state)
{
  core::SimulationState2FWS2FWD simulation_state = toSimulationState2FWS2FWD(
    wheelbase_,
    front_track_,
    rear_track_,
    front_wheel_radius_,
    rear_wheel_radius_,
    front_hub_carrier_offset_,
    rear_hub_carrier_offset_,
    hardware_state);

  rear_left_wheel_spinning_joint_feedback_.set(simulation_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_feedback_.set(simulation_state.rearRightWheelSpinningMotion);
}


}  // namespace ros2
}  // namespace romea
