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
#include <sstream>
#include <string>
#include <vector>

// local
#include "romea_mobile_base_utils/ros2_control/hardware/prismatic_joint_hardware_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
PrismaticJointHardwareInterface::PrismaticJointHardwareInterface(
  const hardware_interface::ComponentInfo & joint_info, const std::string & command_interface_type)
: id_(0), command_(joint_info, command_interface_type), feedback_(joint_info)
{
}

//-----------------------------------------------------------------------------
PrismaticJointHardwareInterface::PrismaticJointHardwareInterface(
  const size_t & joint_id,
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & command_interface_type)
: id_(joint_id), command_(joint_info, command_interface_type), feedback_(joint_info)
{
}

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::export_command_interface(
  std::vector<hardware_interface::CommandInterface> & command_interfaces)
{
  command_.export_interface(command_interfaces);
}

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::export_state_interfaces(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  feedback_.export_state_interfaces(state_interfaces);
}

//-----------------------------------------------------------------------------
PrismaticJointHardwareInterface::Feedback::Feedback(
  const hardware_interface::ComponentInfo & joint_info)
: position(joint_info, hardware_interface::HW_IF_POSITION),
  velocity(joint_info, hardware_interface::HW_IF_VELOCITY),
  force(joint_info, hardware_interface::HW_IF_EFFORT)
{
}

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::Feedback::export_state_interfaces(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  position.export_interface(state_interfaces);
  velocity.export_interface(state_interfaces);
  force.export_interface(state_interfaces);
}

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::Feedback::set(const core::LinearMotionState & state)
{
  position.set(state.position);
  velocity.set(state.velocity);
  force.set(state.force);
}

//-----------------------------------------------------------------------------
core::LinearMotionState PrismaticJointHardwareInterface::Feedback::get() const
{
  core::LinearMotionState state;
  state.position = position.get();
  state.velocity = velocity.get();
  state.force = force.get();
  return state;
}

//-----------------------------------------------------------------------------
double PrismaticJointHardwareInterface::get_command() const
{
  return command_.get();
}

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::set_command(const double & command)
{
  command_.set(command);
}

// //-----------------------------------------------------------------------------
// void PrismaticJointHardwareInterface::set_state(const core::RotationalMotionState & state)
// {
//   feedback_.set_state(state);
// }

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::set_feedback(const core::LinearMotionState & state)
{
  feedback_.set(state);
  // set_state(state);
}

//-----------------------------------------------------------------------------
core::LinearMotionState PrismaticJointHardwareInterface::get_feedback() const
{
  return feedback_.get();
}

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::write_command(
  sensor_msgs::msg::JointState & joint_state_command) const
{
  joint_state_command.name[id_] = get_joint_name();
  if (get_command_type()[0] == 'v') {
    set_velocity(joint_state_command, id_, get_command());
  } else {
    set_effort(joint_state_command, id_, get_command());
  }
}

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::read_feedback(
  const sensor_msgs::msg::JointState & joint_state_feedback)
{
  core::LinearMotionState state;
  auto id = romea::ros2::get_joint_id(joint_state_feedback, get_joint_name());
  state.position = get_position(joint_state_feedback, id);
  state.velocity = get_velocity(joint_state_feedback, id);
  state.force = get_effort(joint_state_feedback, id);
  feedback_.set(state);
}

//-----------------------------------------------------------------------------
void PrismaticJointHardwareInterface::try_read_feedback(
  const sensor_msgs::msg::JointState & joint_state_feedback)
{
  auto id = find_joint_id(joint_state_feedback, get_joint_name());
  if (id.has_value()) {
    core::LinearMotionState state;
    state.position = get_position(joint_state_feedback, id.value());
    state.velocity = get_velocity(joint_state_feedback, id.value());
    state.force = get_effort(joint_state_feedback, id.value());
    feedback_.set(state);
  }
}

//-----------------------------------------------------------------------------
const std::string & PrismaticJointHardwareInterface::get_command_type() const
{
  return command_.get_interface_type();
}

//-----------------------------------------------------------------------------
const std::string & PrismaticJointHardwareInterface::get_joint_name() const
{
  return command_.get_joint_name();
}

//-----------------------------------------------------------------------------
const size_t & PrismaticJointHardwareInterface::get_joint_id() const
{
  return id_;
}

}  // namespace ros2
}  // namespace romea
