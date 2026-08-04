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
#include <cmath>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// romea
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "romea_common_utils/joint_states.hpp"

// local
#include "romea_mobile_base_gazebo/generic_gazebo_interface.hpp"

namespace romea
{
namespace ros2
{

namespace
{

//-----------------------------------------------------------------------------
double get_parameter_or(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & name,
  const double & default_value)
{
  const auto parameter = joint_info.parameters.find(name);
  return parameter == joint_info.parameters.end() ? default_value : std::stod(parameter->second);
}

}  // namespace

//-----------------------------------------------------------------------------
GenericGazeboInterface::GenericGazeboInterface(
  gz::sim::EntityComponentManager & ecm,
  std::map<std::string, gz::sim::Entity> & enable_joints,
  const std::vector<hardware_interface::ComponentInfo> & joint_infos)
: ecm_(&ecm)
{
  joint_names_.reserve(joint_infos.size());
  joint_command_interface_types_.reserve(joint_infos.size());
  sim_joints_.reserve(joint_infos.size());
  positions_.resize(joint_infos.size(), 0.0);
  position_gains_.reserve(joint_infos.size());

  for (const auto & joint_info : joint_infos) {
    auto joint = enable_joints.find(joint_info.name);
    if (joint == enable_joints.end()) {
      std::stringstream msg;
      msg << "Joint called ";
      msg << joint_info.name;
      msg << " is not an available gazebo joint";
      throw std::runtime_error(msg.str());
    }

    if (joint_info.command_interfaces.size() > 1) {
      std::stringstream msg;
      msg << "Gazebo joint ";
      msg << joint_info.name;
      msg << " must have at most one command interface";
      throw std::runtime_error(msg.str());
    }

    const auto command_interface_type = joint_info.command_interfaces.empty() ?
      std::string() :
      joint_info.command_interfaces.front().name;

    joint_names_.push_back(joint_info.name);
    joint_command_interface_types_.push_back(command_interface_type);
    sim_joints_.push_back(joint->second);
    position_gains_.push_back(get_parameter_or(joint_info, "position_gain", 100.0));

    create_state_gazebo_component_<gz::sim::components::JointPosition>(joint->second);
    create_state_gazebo_component_<gz::sim::components::JointVelocity>(joint->second);
    create_state_gazebo_component_<gz::sim::components::JointTransmittedWrench>(joint->second);

    if (
      command_interface_type == hardware_interface::HW_IF_POSITION ||
      command_interface_type == hardware_interface::HW_IF_VELOCITY)
    {
      create_command_gazebo_component_<gz::sim::components::JointVelocityCmd>(joint->second);
    } else if (command_interface_type == hardware_interface::HW_IF_EFFORT) {
      create_command_gazebo_component_<gz::sim::components::JointForceCmd>(joint->second);
    } else if (!command_interface_type.empty()) {
      std::stringstream msg;
      msg << "Unsupported Gazebo joint command interface type ";
      msg << command_interface_type;
      msg << " for joint ";
      msg << joint_info.name;
      throw std::runtime_error(msg.str());
    }
  }
}

//-----------------------------------------------------------------------------
sensor_msgs::msg::JointState GenericGazeboInterface::get_joint_state() const
{
  auto joint_state = make_joint_state_msg(joint_names_);

  for (size_t n = 0; n < sim_joints_.size(); ++n) {
    positions_[n] = get_position_(sim_joints_[n]);
    set_position(joint_state, n, positions_[n]);
    set_velocity(joint_state, n, get_velocity_(sim_joints_[n]));
    set_effort(joint_state, n, get_effort_(sim_joints_[n]));
  }

  return joint_state;
}

//-----------------------------------------------------------------------------
void GenericGazeboInterface::set_command(const sensor_msgs::msg::JointState & command)
{
  for (size_t n = 0; n < joint_names_.size(); ++n) {
    auto command_id = find_joint_id(command, joint_names_[n]);
    if (!command_id.has_value()) {
      continue;
    }

    const auto id = command_id.value();
    const auto & command_interface_type = joint_command_interface_types_[n];

    if (command_interface_type == hardware_interface::HW_IF_VELOCITY) {
      if (id < command.velocity.size() && std::isfinite(command.velocity[id])) {
        set_velocity_command_(sim_joints_[n], command.velocity[id]);
      }
    } else if (command_interface_type == hardware_interface::HW_IF_POSITION) {
      if (id < command.position.size() && std::isfinite(command.position[id])) {
        const auto velocity_command = position_gains_[n] * (command.position[id] - positions_[n]);
        set_velocity_command_(sim_joints_[n], velocity_command);
      }
    } else if (command_interface_type == hardware_interface::HW_IF_EFFORT) {
      if (id < command.effort.size() && std::isfinite(command.effort[id])) {
        set_effort_command_(sim_joints_[n], command.effort[id]);
      }
    }
  }
}

//-----------------------------------------------------------------------------
double GenericGazeboInterface::get_position_(const gz::sim::Entity & sim_joint) const
{
  return ecm_->Component<gz::sim::components::JointPosition>(sim_joint)->Data()[0];
}

//-----------------------------------------------------------------------------
double GenericGazeboInterface::get_velocity_(const gz::sim::Entity & sim_joint) const
{
  return ecm_->Component<gz::sim::components::JointVelocity>(sim_joint)->Data()[0];
}

//-----------------------------------------------------------------------------
double GenericGazeboInterface::get_effort_(const gz::sim::Entity & sim_joint) const
{
  return ecm_->Component<gz::sim::components::JointTransmittedWrench>(sim_joint)
    ->Data()
    .torque()
    .z();
}

//-----------------------------------------------------------------------------
void GenericGazeboInterface::set_velocity_command_(
  const gz::sim::Entity & sim_joint, const double & command)
{
  ecm_->SetComponentData<gz::sim::components::JointVelocityCmd>(sim_joint, {command});
}

//-----------------------------------------------------------------------------
void GenericGazeboInterface::set_effort_command_(
  const gz::sim::Entity & sim_joint, const double & command)
{
  ecm_->SetComponentData<gz::sim::components::JointForceCmd>(sim_joint, {command});
}

}  // namespace ros2
}  // namespace romea
