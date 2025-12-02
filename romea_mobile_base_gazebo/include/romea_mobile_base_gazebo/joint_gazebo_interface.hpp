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


#ifndef ROMEA_MOBILE_BASE_GAZEBO__JOINT_GAZEBO_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__JOINT_GAZEBO_INTERFACE_HPP_

// std
#include <map>
#include <string>

// ros
#include "hardware_interface/hardware_info.hpp"

// gazebo
#include <gz/sim/System.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

namespace romea
{
namespace ros2
{

template <typename Command, typename State>
class JointGazeboInterface
{
public:
  JointGazeboInterface(
    gz::sim::EntityComponentManager & ecm,
    std::map<std::string, gz::sim::Entity> & enable_joints,
    const hardware_interface::ComponentInfo & joint_info):
    ecm_(& ecm)
  {
    // std::cout << " spinning joint_info.name ";
    // std::cout << joint_info.name << std::endl;
    auto it = enable_joints.find(joint_info.name);

    if (it == enable_joints.end()) {
      std::stringstream msg;
      msg << " Joint called ";
      msg << joint_info.name;
      msg << " is not an available gazebo joint";
      throw std::runtime_error(msg.str());
    }

    sim_joint_ = enable_joints[joint_info.name];

    auto joint_type = ecm.Component<gz::sim::components::JointType>(sim_joint_)->Data();
    assert(joint_type == sdf::JointType::REVOLUTE);
  }

public:
  virtual void set_command(const Command & command) = 0;
  virtual State get_state()const = 0;

public:
  template <typename ComponentType>
  void create_state_gazebo_component_()
  {
    if (!ecm_->EntityHasComponentType(sim_joint_, ComponentType().TypeId()))
    {
      ecm_->CreateComponent(sim_joint_, ComponentType());
    }
  }

  template <typename ComponentType>
  void create_command_gazebo_component_()
  {
    if (!ecm_->EntityHasComponentType(sim_joint_, ComponentType().TypeId()))
    {
      ecm_->CreateComponent(sim_joint_, ComponentType({0}));
    }
  }


  template <typename ComponentType>
  double get_state_() const
  {
    if constexpr (std::is_same_v<ComponentType, gz::sim::components::JointTransmittedWrench>)
    {
      return ecm_->Component<ComponentType>(sim_joint_)->Data().torque().z();
    }else{
      return ecm_->Component<ComponentType>(sim_joint_)->Data()[0];
    }
  }

  template <typename ComponentType>
  void set_command_(const double & command)
  {
    ecm_->SetComponentData<ComponentType>(sim_joint_, {command});
    // *ecm_->Component<ComponentType>(sim_joint_) = ComponentType(command);
  }

protected:
  gz::sim::EntityComponentManager * ecm_;
  gz::sim::Entity sim_joint_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__JOINT_GAZEBO_INTERFACE_HPP_
