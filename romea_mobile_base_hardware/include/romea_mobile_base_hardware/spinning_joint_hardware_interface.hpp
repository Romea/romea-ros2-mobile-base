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


#ifndef ROMEA_MOBILE_BASE_HARDWARE__SPINNING_JOINT_HARDWARE_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__SPINNING_JOINT_HARDWARE_INTERFACE_HPP_


// std
#include <string>
#include <vector>

// ros
#include "hardware_interface/hardware_info.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_handle.hpp"
#include "romea_common_utils/joint_states.hpp"

namespace romea
{
namespace ros2
{

core::RotationalMotionControlType toRotationalMotionCommandType(const std::string & interface_type);


class SpinningJointHardwareInterface
{
public:
  using Command = HardwareCommandInterface;
  using CommandType = core::RotationalMotionControlType;

  struct Feedback
  {
    explicit Feedback(const hardware_interface::ComponentInfo & joint_info);
    HardwareStateInterface position;
    HardwareStateInterface velocity;
    HardwareStateInterface torque;

    void set_state(const core::RotationalMotionState & state);

    core::RotationalMotionState get_state()const;

    void export_state_interfaces(
      std::vector<hardware_interface::StateInterface> & state_interfaces);
  };

public:
  SpinningJointHardwareInterface(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & spinning_joint_command_interface_type);

  SpinningJointHardwareInterface(
    const size_t & joint_id,
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & spinning_joint_command_interface_type);

  double get_command() const;
  void set_command(const double & command);

  void set_state(const core::RotationalMotionState & state);
  void set_feedback(const core::RotationalMotionState & state);
  core::RotationalMotionState get_feedback()const;

  void write_command(sensor_msgs::msg::JointState & joint_state_command) const;
  void read_feedback(const sensor_msgs::msg::JointState & joint_state_feedback);
  void try_read_feedback(const sensor_msgs::msg::JointState & joint_state_feedback);

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces);
  void export_state_interfaces(
    std::vector<hardware_interface::StateInterface> & state_interfaces);

  const std::string & get_command_type() const;
  const std::string & get_joint_name() const;
  const size_t & get_joint_id() const;

private:
  size_t id_;
  Command command_;
  Feedback feedback_;
};

// void write_command(
//   const core::RotationalMotionCommand & command;
//   const SpinningJointHardwareInterface & joint,
//   sensor_msgs::msg::JointState & joint_state_command);

// void read_feedback(
//   const sensor_msgs::msg::JointState & joint_state_feedback,
//   const SpinningJointHardwareInterface & joint,
//   core::RotationalMotionState & state);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__SPINNING_JOINT_HARDWARE_INTERFACE_HPP_
