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
#include <string>

// romea
#include "romea_core_mobile_base/hardware/HardwareControlCommon.hpp"

// ros
#include "hardware_interface/hardware_info.hpp"

// gazebo
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"


namespace romea
{
namespace ros2
{

class SpinningJointGazeboInterface
{
public:
  SpinningJointGazeboInterface(
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & command_interface_type);

  void set_command(const double & command);
  core::RotationalMotionState get_state()const;

private:
  core::RotationalMotionControlType control_type;
  gazebo::physics::JointPtr sim_joint_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__SPINNING_JOINT_GAZEBO_INTERFACE_HPP_
