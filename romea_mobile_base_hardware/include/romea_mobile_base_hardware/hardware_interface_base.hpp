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

#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE_BASE_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE_BASE_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// romea
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace romea
{
namespace ros2
{

class HardwareInterfaceBase
{
public:
  virtual ~HardwareInterfaceBase() = default;

  // virtual const std::string & name() const = 0;
  // virtual const std::string & type() const = 0;

  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() = 0;

  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() = 0;

  std::vector<std::string> get_joint_names();

  virtual sensor_msgs::msg::JointState get_joint_state_command() = 0;

  virtual void set_feedback(const sensor_msgs::msg::JointState & joint_states) = 0;
};

std::unique_ptr<HardwareInterfaceBase> make_hardware_interface(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & interface_type,
  const std::string & parameters_prefix);

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE_BASE_HPP_
