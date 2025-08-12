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


#ifndef ROMEA_MOBILE_BASE_GAZEBO__SPINNING_JOINT_GAZEBO_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__SPINNING_JOINT_GAZEBO_INTERFACE_HPP_

// std
#include <map>
#include <string>

// romea
#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"


// local
#include "romea_mobile_base_gazebo/joint_gazebo_interface.hpp"

namespace romea
{
namespace ros2
{

class SpinningJointGazeboInterface :
  public JointGazeboInterface<double, core::RotationalMotionState>
{
public:
  SpinningJointGazeboInterface(
    gz::sim::EntityComponentManager & ecm,
    std::map<std::string, gz::sim::Entity> & enable_joints,
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & command_interface_type);

  void set_command(const double & command) final;
  core::RotationalMotionState get_state()const final;

private:
  core::RotationalMotionControlType control_type;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__SPINNING_JOINT_GAZEBO_INTERFACE_HPP_
