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


#ifndef ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2TTD_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2TTD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"
#include "romea_core_mobile_base/simulation/SimulationControl2TTD.hpp"


namespace romea
{
namespace ros2
{

class SimulationInterface2TTD
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

  SimulationInterface2TTD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  core::SimulationCommand2TTD get_hardware_command();
  sensor_msgs::msg::JointState get_joint_state_command();

  void set_feedback(const core::SimulationState2TTD & simulation_state);
  void set_feedback(const sensor_msgs::msg::JointState & joint_states);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SpinningJointHardwareInterface left_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface right_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface left_idler_wheel_spinning_joint_;
  SpinningJointHardwareInterface right_idler_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_left_roller_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_roller_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_left_roller_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_roller_wheel_spinning_joint_;

  const double idler_wheel_radius_;
  const double roller_wheel_radius_;
  const double sprocket_wheel_radius_;
  const double track_thickness_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2TTD_HPP_
