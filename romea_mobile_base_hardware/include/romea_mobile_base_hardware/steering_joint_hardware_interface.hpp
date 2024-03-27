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


#ifndef ROMEA_MOBILE_BASE_HARDWARE__STEERING_JOINT_HARDWARE_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__STEERING_JOINT_HARDWARE_INTERFACE_HPP_

// std
#include <vector>
#include <string>

// ros
#include "hardware_interface/hardware_info.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_handle.hpp"
#include "romea_common_utils/joint_states.hpp"

namespace romea
{
namespace ros2
{

class SteeringJointHardwareInterface
{
public:
  using Command = HardwareCommandInterface;
  using Feedback = HardwareStateInterface;

public:
  explicit SteeringJointHardwareInterface(const hardware_interface::ComponentInfo & joint_info);

  SteeringJointHardwareInterface(
    const size_t & joint_id,
    const hardware_interface::ComponentInfo & joint_info
  );

  core::SteeringAngleCommand get_command() const;
  void set_command(const core::SteeringAngleCommand & command);


  void set_state(const core::SteeringAngleState & state);
  void set_feedback(const core::SteeringAngleState & state);
  core::SteeringAngleState get_feedback()const;

  void write_command(sensor_msgs::msg::JointState & joint_state_command) const;
  void read_feedback(const sensor_msgs::msg::JointState & joint_state_feedback);
  void try_read_feedback(const sensor_msgs::msg::JointState & joint_state_feedback);

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & hardware_interfaces);
  void export_state_interface(
    std::vector<hardware_interface::StateInterface> & hardware_interfaces);

  const std::string & get_command_type() const;
  const std::string & get_joint_name() const;
  const size_t & get_joint_id() const;

private:
  const size_t id_;
  Command command_;
  Feedback feedback_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__STEERING_JOINT_HARDWARE_INTERFACE_HPP_
