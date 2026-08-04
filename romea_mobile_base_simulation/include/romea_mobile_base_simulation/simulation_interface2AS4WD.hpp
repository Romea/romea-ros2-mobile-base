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

#ifndef ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2AS4WD_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2AS4WD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl2AS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface_base.hpp"

namespace romea
{
namespace ros2
{

class SimulationInterface2AS4WD final : public SimulationInterfaceBase
{
public:
  struct Configuration
  {
    Configuration() = default;

    Configuration(
      const hardware_interface::HardwareInfo & hardware_info,
      const std::string & parameters_prefix);

    hardware_interface::ComponentInfo front_axle_steering_joint_info;
    hardware_interface::ComponentInfo rear_axle_steering_joint_info;
    hardware_interface::ComponentInfo front_left_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo front_right_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo rear_left_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo rear_right_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo front_left_wheel_steering_joint_info;
    hardware_interface::ComponentInfo front_right_wheel_steering_joint_info;
    hardware_interface::ComponentInfo rear_left_wheel_steering_joint_info;
    hardware_interface::ComponentInfo rear_right_wheel_steering_joint_info;
    std::string spinning_joint_command_interface_type;
    double wheelbase;
    double front_track;
    double rear_track;
  };

  enum JointIds
  {
    FRONT_AXLE_STEERING_JOINT_ID = 0,
    REAR_AXLE_STEERING_JOINT_ID = 1,
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID = 2,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID = 3,
    REAR_LEFT_WHEEL_STEERING_JOINT_ID = 4,
    REAR_RIGHT_WHEEL_STEERING_JOINT_ID = 5,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID = 6,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID = 7,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID = 8,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID = 9
  };

  SimulationInterface2AS4WD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type);

  SimulationInterface2AS4WD(const Configuration & configuration);

  core::SimulationCommand2AS4WD get_hardware_command();
  sensor_msgs::msg::JointState get_joint_state_command() override;

  void set_feedback(const core::SimulationState2AS4WD & simulation_state);
  void set_feedback(const sensor_msgs::msg::JointState & joint_states) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  SteeringJointHardwareInterface front_axle_steering_joint_;
  SteeringJointHardwareInterface rear_axle_steering_joint_;
  SpinningJointHardwareInterface front_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;
  SteeringJointHardwareInterface front_left_wheel_steering_joint_;
  SteeringJointHardwareInterface front_right_wheel_steering_joint_;
  SteeringJointHardwareInterface rear_left_wheel_steering_joint_;
  SteeringJointHardwareInterface rear_right_wheel_steering_joint_;

  const double wheelbase_;
  const double front_track_;
  const double rear_track_;
};

std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const SimulationInterface2AS4WD::Configuration & configuration);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2AS4WD_HPP_
