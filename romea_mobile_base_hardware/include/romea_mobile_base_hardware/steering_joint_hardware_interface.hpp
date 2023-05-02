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

// ros
#include "hardware_interface/hardware_info.hpp"

// local
#include "hardware_handle.hpp"

namespace romea
{

class SteeringJointHardwareInterface
{
public:
  using Command = HardwareCommandInterface;
  using Feedback = HardwareStateInterface;

public:
  explicit SteeringJointHardwareInterface(const hardware_interface::ComponentInfo & joint_info);

  SteeringAngleCommand get_command() const;
  void set_state(const SteeringAngleState & state);

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & hardware_interfaces);
  void export_state_interface(
    std::vector<hardware_interface::StateInterface> & hardware_interfaces);

private:
  Command command_;
  Feedback feedback_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__STEERING_JOINT_HARDWARE_INTERFACE_HPP_
