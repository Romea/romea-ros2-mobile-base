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
#include <sstream>

// local
#include "romea_mobile_base_utils/ros2_control/hardware/spinning_joint_hardware_interface.hpp"

namespace romea
{
namespace ros2
{

core::RotationalMotionControlType toRotationalMotionCommandType(const std::string & interface_type)
{
  if (interface_type == hardware_interface::HW_IF_VELOCITY) {
    return core::RotationalMotionControlType::VELOCITY;
  } else if (interface_type == hardware_interface::HW_IF_EFFORT) {
    return core::RotationalMotionControlType::TORQUE;
  } else {
    std::stringstream msg;
    msg << "Unable to convert interface type ";
    msg << interface_type;
    msg << "to rotational motion control type ";
    throw std::runtime_error(msg.str());
  }
}

//-----------------------------------------------------------------------------
SpinningJointHardwareInterface::SpinningJointHardwareInterface(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & spinning_joint_command_interface_type)
: id_(0),
  command_(joint_info, spinning_joint_command_interface_type),
  feedback_(joint_info)
{
}

//-----------------------------------------------------------------------------
SpinningJointHardwareInterface::SpinningJointHardwareInterface(
  const size_t & joint_id,
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & spinning_joint_command_interface_type)
: id_(joint_id),
  command_(joint_info, spinning_joint_command_interface_type),
  feedback_(joint_info)
{
}


//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::export_command_interface(
  std::vector<hardware_interface::CommandInterface> & command_interfaces)
{
  command_.export_interface(command_interfaces);
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::export_state_interfaces(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  feedback_.export_state_interfaces(state_interfaces);
}

//-----------------------------------------------------------------------------
SpinningJointHardwareInterface::Feedback::Feedback(
  const hardware_interface::ComponentInfo & joint_info)
: position(joint_info, hardware_interface::HW_IF_POSITION),
  velocity(joint_info, hardware_interface::HW_IF_VELOCITY),
  torque(joint_info, hardware_interface::HW_IF_EFFORT)
{
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::Feedback::export_state_interfaces(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  position.export_interface(state_interfaces);
  velocity.export_interface(state_interfaces);
  torque.export_interface(state_interfaces);
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::Feedback::set(const core::RotationalMotionState & state)
{
  position.set(state.position);
  velocity.set(state.velocity);
  torque.set(state.torque);
}

//-----------------------------------------------------------------------------
core::RotationalMotionState SpinningJointHardwareInterface::Feedback::get()const
{
  core::RotationalMotionState state;
  state.position = position.get();
  state.velocity = velocity.get();
  state.torque = torque.get();
  return state;
}

//-----------------------------------------------------------------------------
double SpinningJointHardwareInterface::get_command() const
{
  return command_.get();
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::set_command(const double & command)
{
  command_.set(command);
}

// //-----------------------------------------------------------------------------
// void SpinningJointHardwareInterface::set_state(const core::RotationalMotionState & state)
// {
//   feedback_.set_state(state);
// }

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::set_feedback(const core::RotationalMotionState & state)
{
  feedback_.set(state);
  // set_state(state);
}

//-----------------------------------------------------------------------------
core::RotationalMotionState SpinningJointHardwareInterface::get_feedback()const
{
  return feedback_.get();
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::write_command(
  sensor_msgs::msg::JointState & joint_state_command)const
{
  joint_state_command.name[id_] = get_joint_name();
  if (get_command_type()[0] == 'v') {
    set_velocity(joint_state_command, id_, get_command());
  } else {
    set_effort(joint_state_command, id_, get_command());
  }
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::read_feedback(
  const sensor_msgs::msg::JointState & joint_state_feedback)
{
  core::RotationalMotionState state;
  auto id = romea::ros2::get_joint_id(joint_state_feedback, get_joint_name());
  state.position = get_position(joint_state_feedback, id);
  state.velocity = get_velocity(joint_state_feedback, id);
  state.torque = get_effort(joint_state_feedback, id);
  feedback_.set(state);
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::try_read_feedback(
  const sensor_msgs::msg::JointState & joint_state_feedback)
{
  auto id = find_joint_id(joint_state_feedback, get_joint_name());
  if (id.has_value()) {
    core::RotationalMotionState state;
    state.position = get_position(joint_state_feedback, id.value());
    state.velocity = get_velocity(joint_state_feedback, id.value());
    state.torque = get_effort(joint_state_feedback, id.value());
    feedback_.set(state);
  }
}

//-----------------------------------------------------------------------------
const std::string & SpinningJointHardwareInterface::get_command_type() const
{
  return command_.get_interface_type();
}

//-----------------------------------------------------------------------------
const std::string & SpinningJointHardwareInterface::get_joint_name() const
{
  return command_.get_joint_name();
}

//-----------------------------------------------------------------------------
const size_t & SpinningJointHardwareInterface::get_joint_id() const
{
  return id_;
}

}  // namespace ros2
}  // namespace romea
