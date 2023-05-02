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


#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2TTD_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2TTD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

// std
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"

namespace romea
{

class HardwareInterface2TTD
{
public:
  enum JointIDs
  {
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID = 0,
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID = 1,
    LEFT_IDLER_WHEEL_SPINNING_JOINT_ID = 2,
    RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID = 3,
    FRONT_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID = 4,
    FRONT_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID = 5,
    REAR_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID = 6,
    REAR_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID = 7
  };

  HardwareInterface2TTD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);


  HardwareCommand2TD get_command()const;

  void set_state(const HardwareState2TD & hardware_state);

  void set_state(
    const HardwareState2TD & hardware_state,
    const RotationalMotionState & left_idler_wheel_spinning_motion,
    const RotationalMotionState & right_idler_wheel_spinning_motion,
    const RotationalMotionState & front_left_roller_wheel_spinning_motion,
    const RotationalMotionState & front_right_roller_wheel_spinning_motion,
    const RotationalMotionState & rear_left_roller_wheel_spinning_motion,
    const RotationalMotionState & rear_right_roller_wheel_spinning_motion);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SpinningJointHardwareInterface left_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface right_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface::Feedback left_idler_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback right_idler_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback front_left_roller_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback front_right_roller_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback rear_left_roller_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback rear_right_roller_wheel_spinning_joint_feedback_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2TTD_HPP_
