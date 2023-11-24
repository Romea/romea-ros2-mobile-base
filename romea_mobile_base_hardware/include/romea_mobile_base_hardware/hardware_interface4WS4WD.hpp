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


#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WS4WD_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WS4WD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/hardware/HardwareControl4WS4WD.hpp"

// local
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"
#include "romea_mobile_base_hardware/steering_joint_hardware_interface.hpp"


namespace romea
{
namespace ros2
{

class HardwareInterface4WS4WD
{
public:
  enum JointIDs
  {
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID = 0,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID = 1,
    REAR_LEFT_WHEEL_STEERING_JOINT_ID = 2,
    REAR_RIGHT_WHEEL_STEERING_JOINT_ID = 3,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID = 4,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID = 5,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID = 6,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID = 7
  };

  HardwareInterface4WS4WD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type);

  core::HardwareCommand4WS4WD get_command()const;
  void set_state(const core::HardwareState4WS4WD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SteeringJointHardwareInterface front_left_wheel_steering_joint_;
  SteeringJointHardwareInterface front_right_wheel_steering_joint_;
  SteeringJointHardwareInterface rear_left_wheel_steering_joint_;
  SteeringJointHardwareInterface rear_right_wheel_steering_joint_;
  SpinningJointHardwareInterface front_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WS4WD_HPP_
