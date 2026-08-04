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

#ifndef ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2THD_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2THD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl2THD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2THD.hpp"
#include "romea_mobile_base_simulation/simulation_interface_base.hpp"

namespace romea
{
namespace ros2
{

class SimulationInterface2THD final : public SimulationInterfaceBase
{
public:
  struct Configuration
  {
    Configuration() = default;

    Configuration(
      const hardware_interface::HardwareInfo & hardware_info,
      const std::string & parameters_prefix);

    hardware_interface::ComponentInfo left_sprocket_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo right_sprocket_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo front_left_idler_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo front_right_idler_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo rear_left_idler_wheel_spinning_joint_info;
    hardware_interface::ComponentInfo rear_right_idler_wheel_spinning_joint_info;
    std::string spinning_joint_command_interface_type;
    double idler_wheel_radius;
    double sprocket_wheel_radius;
    double track_thickness;
  };

  enum JointIDs
  {
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID = 0,
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID = 1,
    FRONT_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID = 2,
    FRONT_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID = 3,
    REAR_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID = 4,
    REAR_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID = 5
  };

  SimulationInterface2THD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  SimulationInterface2THD(const Configuration & configuration);

  core::SimulationCommand2THD get_hardware_command();
  sensor_msgs::msg::JointState get_joint_state_command() override;

  void set_feedback(const core::SimulationState2THD & simulation_state);
  void set_feedback(const sensor_msgs::msg::JointState & joint_states) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  SpinningJointHardwareInterface left_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface right_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_left_idler_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_idler_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_left_idler_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_idler_wheel_spinning_joint_;

  const double idler_wheel_radius_;
  const double sprocket_wheel_radius_;
  const double track_thickness_;
};

std::vector<hardware_interface::ComponentInfo> get_gazebo_joint_infos(
  const SimulationInterface2THD::Configuration & configuration);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2THD_HPP_
