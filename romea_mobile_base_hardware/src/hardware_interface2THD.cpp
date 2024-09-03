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
#include "romea_core_mobile_base/simulation/SimulationControl2THD.hpp"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2THD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2THD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HardwareInterface2THD::HardwareInterface2THD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: left_sprocket_wheel_spinning_joint_(
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo2THD::get_left_sprocket_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  right_sprocket_wheel_spinning_joint_(
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    HardwareInfo2THD::get_right_sprocket_wheel_spinning_joint_info(hardware_info),
    command_interface_type),
  front_left_idler_wheel_spinning_joint_feedback_(
    HardwareInfo2THD::get_front_left_idler_wheel_spinning_joint_info(hardware_info)),
  front_right_idler_wheel_spinning_joint_feedback_(
    HardwareInfo2THD::get_front_right_idler_wheel_spinning_joint_info(hardware_info)),
  rear_left_idler_wheel_spinning_joint_feedback_(
    HardwareInfo2THD::get_rear_left_idler_wheel_spinning_joint_info(hardware_info)),
  rear_right_idler_wheel_spinning_joint_feedback_(
    HardwareInfo2THD::get_rear_right_idler_wheel_spinning_joint_info(hardware_info)),
  sprocket_wheel_radius_(get_sprocket_wheel_radius(hardware_info)),
  idler_wheel_radius_(get_idler_wheel_radius(hardware_info)),
  track_thickness_(get_track_thickness(hardware_info))
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2THD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_left_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  front_right_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_left_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_right_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2THD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
core::HardwareCommand2TD HardwareInterface2THD::get_hardware_command() const
{
  // *INDENT-OFF*
  return {left_sprocket_wheel_spinning_joint_.get_command(),
      right_sprocket_wheel_spinning_joint_.get_command()};
  // *INDENT-ON*

  // return get_command();
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState HardwareInterface2THD::get_joint_state_command() const
{
  auto joint_states = make_joint_state_msg(2);
  left_sprocket_wheel_spinning_joint_.write_command(joint_states);
  right_sprocket_wheel_spinning_joint_.write_command(joint_states);
  return joint_states;
}

//-----------------------------------------------------------------------------
void HardwareInterface2THD::set_feedback(const core::HardwareState2TD & hardware_state)
{
  left_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.leftSprocketWheelSpinningMotion);
  right_sprocket_wheel_spinning_joint_.set_feedback(
    hardware_state.rightSprocketWheelSpinningMotion);
  // complete_feedback_(hardware_state);
}

//-----------------------------------------------------------------------------
void HardwareInterface2THD::set_feedback(const sensor_msgs::msg::JointState & joint_states)
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
void HardwareInterface2THD::complete_feedback_(const core::HardwareState2TD & hardware_state)
{
  core::SimulationState2THD simulation_state = toSimulationState2THD(
    sprocket_wheel_radius_,
    idler_wheel_radius_,
    track_thickness_,
    hardware_state);

  front_left_idler_wheel_spinning_joint_feedback_.set(
    simulation_state.frontLeftIdlerWheelSpinningMotion);
  front_right_idler_wheel_spinning_joint_feedback_.set(
    simulation_state.frontRightIdlerWheelSpinningMotion);
  rear_left_idler_wheel_spinning_joint_feedback_.set(
    simulation_state.rearLeftIdlerWheelSpinningMotion);
  rear_right_idler_wheel_spinning_joint_feedback_.set(
    simulation_state.rearRightIdlerWheelSpinningMotion);
}

}  // namespace ros2
}  // namespace romea
