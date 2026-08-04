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

#ifndef ROMEA_MOBILE_BASE_GAZEBO__GENERIC_GAZEBO_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__GENERIC_GAZEBO_INTERFACE_HPP_

// std
#include <map>
#include <string>
#include <vector>

// ros
#include "hardware_interface/hardware_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// gazebo
#include <gz/sim/System.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

namespace romea
{
namespace ros2
{

class GenericGazeboInterface
{
public:
  GenericGazeboInterface(
    gz::sim::EntityComponentManager & ecm,
    std::map<std::string, gz::sim::Entity> & enable_joints,
    const std::vector<hardware_interface::ComponentInfo> & joint_infos);

  sensor_msgs::msg::JointState get_joint_state() const;

  void set_command(const sensor_msgs::msg::JointState & command);

private:
  template<typename ComponentType>
  void create_state_gazebo_component_(const gz::sim::Entity & sim_joint)
  {
    if (!ecm_->EntityHasComponentType(sim_joint, ComponentType().TypeId())) {
      ecm_->CreateComponent(sim_joint, ComponentType());
    }
  }

  template<typename ComponentType>
  void create_command_gazebo_component_(const gz::sim::Entity & sim_joint)
  {
    if (!ecm_->EntityHasComponentType(sim_joint, ComponentType().TypeId())) {
      ecm_->CreateComponent(sim_joint, ComponentType({0}));
    }
  }

  double get_position_(const gz::sim::Entity & sim_joint) const;

  double get_velocity_(const gz::sim::Entity & sim_joint) const;

  double get_effort_(const gz::sim::Entity & sim_joint) const;

  void set_velocity_command_(const gz::sim::Entity & sim_joint, const double & command);

  void set_effort_command_(const gz::sim::Entity & sim_joint, const double & command);

private:
  gz::sim::EntityComponentManager * ecm_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> joint_command_interface_types_;
  std::vector<gz::sim::Entity> sim_joints_;
  mutable std::vector<double> positions_;
  std::vector<double> position_gains_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__GENERIC_GAZEBO_INTERFACE_HPP_
