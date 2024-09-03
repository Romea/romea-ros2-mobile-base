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
#include "romea_mobile_base_utils/ros2_control/hardware/spinning_joint_hardware_interface.hpp"

namespace romea
{
namespace ros2
{

class HardwareInterface2TTD
{
public:
  enum JointIDs
  {
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID = 0,
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID = 1,
  };

  HardwareInterface2TTD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  core::HardwareCommand2TD get_hardware_command() const;
  sensor_msgs::msg::JointState get_joint_state_command() const;

  void set_feedback(const core::HardwareState2TD & hardware_state);
  void set_feedback(const sensor_msgs::msg::JointState & joint_states);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  void complete_feedback_(const core::HardwareState2TD & hardware_state);

private:
  SpinningJointHardwareInterface left_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface right_sprocket_wheel_spinning_joint_;

  SpinningJointHardwareInterface::Feedback left_idler_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback right_idler_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback front_left_roller_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback front_right_roller_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback rear_left_roller_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback rear_right_roller_wheel_spinning_joint_feedback_;

  const double idler_wheel_radius_;
  const double roller_wheel_radius_;
  const double sprocket_wheel_radius_;
  const double track_thickness_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2TTD_HPP_
