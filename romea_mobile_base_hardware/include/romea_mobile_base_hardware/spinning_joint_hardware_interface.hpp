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

// local
#include "romea_mobile_base_hardware/hardware_handle.hpp"

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

    void export_state_interfaces(
      std::vector<hardware_interface::StateInterface> & state_interfaces);
  };

public:
  SpinningJointHardwareInterface(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & spinning_joint_command_interface_type);

  double get_command() const;
  void set_state(const core::RotationalMotionState & state);

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces);
  void export_state_interfaces(std::vector<hardware_interface::StateInterface> & state_interfaces);

private:
  Command command_;
  Feedback feedback_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__SPINNING_JOINT_HARDWARE_INTERFACE_HPP_
