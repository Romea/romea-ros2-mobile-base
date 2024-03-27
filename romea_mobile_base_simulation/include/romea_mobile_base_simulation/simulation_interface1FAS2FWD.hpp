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


#ifndef ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE1FAS2FWD_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE1FAS2FWD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_hardware/hardware_interface1FAS2FWD.hpp"
#include "romea_core_mobile_base/simulation/SimulationControl1FAS2FWD.hpp"


namespace romea
{
namespace ros2
{


class SimulationInterface1FAS2FWD
{
public:
  enum JointIds
  {
    FRONT_AXLE_STEERING_JOINT_ID = 0,
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID = 1,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID = 2,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID = 3,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID = 4,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID = 5,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID = 6
  };

  SimulationInterface1FAS2FWD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type);

  // core::SimulationCommand1FAS2FWD get_command()const;
  // void set_state(const core::SimulationState1FAS2FWD & hardware_state);

  core::SimulationCommand1FAS2FWD get_hardware_command();
  sensor_msgs::msg::JointState get_joint_state_command();

  void set_feedback(const core::SimulationState1FAS2FWD & simulation_state);
  void set_feedback(const sensor_msgs::msg::JointState & joint_states);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SteeringJointHardwareInterface front_axle_steering_joint_;
  SpinningJointHardwareInterface front_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint_;

  SteeringJointHardwareInterface front_left_wheel_steering_joint_;
  SteeringJointHardwareInterface front_right_wheel_steering_joint_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;

  const double wheelbase_;
  const double front_track_;
  const double front_wheel_radius_;
  const double front_hub_carrier_offset_;
  const double rear_track_;
  const double rear_wheel_radius_;
  const double rear_hub_carrier_offset_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE1FAS2FWD_HPP_
