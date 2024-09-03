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
#include <vector>
#include <string>

// local
#include "romea_mobile_base_utils/ros2_control/hardware/steering_joint_hardware_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SteeringJointHardwareInterface::SteeringJointHardwareInterface(
  const hardware_interface::ComponentInfo & joint_info)
: id_(0),
  command_(joint_info, hardware_interface::HW_IF_POSITION),
  feedback_(joint_info, hardware_interface::HW_IF_POSITION)
{
}

//-----------------------------------------------------------------------------
SteeringJointHardwareInterface::SteeringJointHardwareInterface(
  const size_t & joint_id, const hardware_interface::ComponentInfo & joint_info)
: id_(joint_id),
  command_(joint_info, hardware_interface::HW_IF_POSITION),
  feedback_(joint_info, hardware_interface::HW_IF_POSITION)
{
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::export_command_interface(
  std::vector<hardware_interface::CommandInterface> & command_interfaces)
{
  command_.export_interface(command_interfaces);
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::export_state_interface(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  feedback_.export_interface(state_interfaces);
}

//-----------------------------------------------------------------------------
core::SteeringAngleCommand SteeringJointHardwareInterface::get_command()const
{
  return command_.get();
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::set_command(const core::SteeringAngleCommand & command)
{
  command_.set(command);
}

// //-----------------------------------------------------------------------------
// void SteeringJointHardwareInterface::set_state(const core::SteeringAngleState & state)
// {
//   feedback_.set(state);
// }

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::set_feedback(const core::SteeringAngleState & state)
{
  feedback_.set(state);
  // set_state(state);
}

//-----------------------------------------------------------------------------
core::SteeringAngleState SteeringJointHardwareInterface::get_feedback()const
{
  return feedback_.get();
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::write_command(
  sensor_msgs::msg::JointState & joint_state_command)const
{
  joint_state_command.name[id_] = get_joint_name();
  set_position(joint_state_command, id_, get_command());
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::read_feedback(
  const sensor_msgs::msg::JointState & joint_state_feedback)
{
  auto id = romea::ros2::get_joint_id(joint_state_feedback, get_joint_name());
  feedback_.set(get_position(joint_state_feedback, id));
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::try_read_feedback(
  const sensor_msgs::msg::JointState & joint_state_feedback)
{
  auto id = find_joint_id(joint_state_feedback, get_joint_name());
  if (id.has_value()) {
    feedback_.set(get_position(joint_state_feedback, id.value()));
  }
}

//-----------------------------------------------------------------------------
const std::string & SteeringJointHardwareInterface::get_command_type() const
{
  return command_.get_interface_type();
}

//-----------------------------------------------------------------------------
const std::string & SteeringJointHardwareInterface::get_joint_name() const
{
  return command_.get_joint_name();
}

//-----------------------------------------------------------------------------
const size_t & SteeringJointHardwareInterface::get_joint_id() const
{
  return id_;
}

}  // namespace ros2
}  // namespace romea
