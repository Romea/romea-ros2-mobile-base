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


#ifndef ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE4WS4WD_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE4WS4WD_HPP_

// std
#include <string>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl4WS4WD.hpp"

// local
#include "romea_mobile_base_gazebo/spinning_joint_gazebo_interface.hpp"
#include "romea_mobile_base_gazebo/steering_joint_gazebo_interface.hpp"

namespace romea
{

class GazeboInterface4WS4WD
{
public:
  GazeboInterface4WS4WD(
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  SimulationState4WS4WD get_state() const;
  void set_command(const SimulationCommand4WS4WD & command);

private:
  SteeringJointGazeboInterface front_left_wheel_steering_joint_;
  SteeringJointGazeboInterface front_right_wheel_steering_joint_;
  SteeringJointGazeboInterface rear_left_wheel_steering_joint_;
  SteeringJointGazeboInterface rear_right_wheel_steering_joint_;
  SpinningJointGazeboInterface front_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE4WS4WD_HPP_
