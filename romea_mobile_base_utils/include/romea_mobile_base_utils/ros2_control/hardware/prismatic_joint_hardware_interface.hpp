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

#ifndef ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__HARDWARE__PRISMATIC_JOINT_HARDWARE_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__HARDWARE__PRISMATIC_JOINT_HARDWARE_INTERFACE_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_common_utils/joint_states.hpp"
#include "romea_mobile_base_utils/ros2_control/hardware/hardware_handle.hpp"

namespace romea
{
namespace ros2
{

class PrismaticJointHardwareInterface
{
public:
  using Command = HardwareCommandInterface;

  struct Feedback
  {
    explicit Feedback(const hardware_interface::ComponentInfo & joint_info);
    HardwareStateInterface position;
    HardwareStateInterface velocity;
    HardwareStateInterface force;

    void set(const core::LinearMotionState & state);

    core::LinearMotionState get() const;

    void export_state_interfaces(
      std::vector<hardware_interface::StateInterface> & state_interfaces);
  };

public:
  PrismaticJointHardwareInterface(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & command_interface_type);

  PrismaticJointHardwareInterface(
    const size_t & joint_id,
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & command_interface_type);

  double get_command() const;
  void set_command(const double & command);

  void set_feedback(const core::LinearMotionState & state);
  core::LinearMotionState get_feedback() const;

  void write_command(sensor_msgs::msg::JointState & joint_state_command) const;
  void read_feedback(const sensor_msgs::msg::JointState & joint_state_feedback);
  void try_read_feedback(const sensor_msgs::msg::JointState & joint_state_feedback);

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces);
  void export_state_interfaces(std::vector<hardware_interface::StateInterface> & state_interfaces);

  const std::string & get_command_type() const;
  const std::string & get_joint_name() const;
  const size_t & get_joint_id() const;

private:
  size_t id_;
  Command command_;
  Feedback feedback_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__HARDWARE__PRISMATIC_JOINT_HARDWARE_INTERFACE_HPP_
