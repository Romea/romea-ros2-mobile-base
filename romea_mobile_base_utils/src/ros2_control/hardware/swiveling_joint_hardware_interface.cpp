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
#include "romea_mobile_base_utils/ros2_control/hardware/swiveling_joint_hardware_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SwivelingJointHardwareInterface::SwivelingJointHardwareInterface(
  const hardware_interface::ComponentInfo & joint_info)
: id_(0), feedback_(joint_info, hardware_interface::HW_IF_POSITION)
{
}

//-----------------------------------------------------------------------------
SwivelingJointHardwareInterface::SwivelingJointHardwareInterface(
  const size_t & joint_id, const hardware_interface::ComponentInfo & joint_info)
: id_(joint_id), feedback_(joint_info, hardware_interface::HW_IF_POSITION)
{
}

//-----------------------------------------------------------------------------
void SwivelingJointHardwareInterface::set_feedback(const core::SteeringAngleState & state)
{
  feedback_.set(state);
}

//-----------------------------------------------------------------------------
core::SteeringAngleState SwivelingJointHardwareInterface::get_feedback() const
{
  return feedback_.get();
}

//-----------------------------------------------------------------------------
void SwivelingJointHardwareInterface::read_feedback(
  const sensor_msgs::msg::JointState & joint_state_feedback)
{
  auto id = romea::ros2::get_joint_id(joint_state_feedback, get_joint_name());
  feedback_.set(get_position(joint_state_feedback, id));
}

//-----------------------------------------------------------------------------
void SwivelingJointHardwareInterface::try_read_feedback(
  const sensor_msgs::msg::JointState & joint_state_feedback)
{
  auto id = find_joint_id(joint_state_feedback, get_joint_name());
  if (id.has_value()) {
    feedback_.set(get_position(joint_state_feedback, id.value()));
  }
}

//-----------------------------------------------------------------------------
void SwivelingJointHardwareInterface::export_state_interface(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  feedback_.export_interface(state_interfaces);
}

//-----------------------------------------------------------------------------
const std::string & SwivelingJointHardwareInterface::get_joint_name() const
{
  return feedback_.get_joint_name();
}

//-----------------------------------------------------------------------------
const size_t & SwivelingJointHardwareInterface::get_joint_id() const
{
  return id_;
}

}  // namespace ros2
}  // namespace romea
