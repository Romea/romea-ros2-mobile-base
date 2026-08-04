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

#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2FWC2RWD_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2FWC2RWD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/hardware/HardwareControl2FWC2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface_base.hpp"
#include "romea_mobile_base_utils/ros2_control/hardware/spinning_joint_hardware_interface.hpp"
#include "romea_mobile_base_utils/ros2_control/hardware/swiveling_joint_hardware_interface.hpp"

namespace romea
{
namespace ros2
{

class HardwareInterface2FWC2RWD final : public HardwareInterfaceBase
{
public:
  struct Configuration
  {
    Configuration() = default;

    Configuration(
      const hardware_interface::HardwareInfo & hardware_info,
      const std::string & parameters_prefix);

    hardware_interface::ComponentInfo front_left_wheel_swiveling_joint_info;
    hardware_interface::ComponentInfo front_right_wheel_swiveling_joint_info;
    hardware_interface::ComponentInfo front_left_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo front_right_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo rear_left_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo rear_right_wheel_spinning_joint_info;
    std::string spinning_joint_command_interface_type;
  };

  enum JointIDs
  {
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID = 0,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID = 1
  };

  HardwareInterface2FWC2RWD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type);

  HardwareInterface2FWC2RWD(const Configuration & configuration);

  core::HardwareCommand2FWC2RWD get_hardware_command() const;
  sensor_msgs::msg::JointState get_joint_state_command();

  void set_feedback(const sensor_msgs::msg::JointState & joint_states);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SwivelingJointHardwareInterface front_left_wheel_swiveling_joint_;
  SwivelingJointHardwareInterface front_right_wheel_swiveling_joint_;
  SpinningJointHardwareInterface::Feedback front_left_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback front_right_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2FWC2RWD_HPP_
