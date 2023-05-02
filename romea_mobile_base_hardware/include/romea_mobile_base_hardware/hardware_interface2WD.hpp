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


#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2WD_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2WD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/hardware/HardwareControl2WD.hpp"

// local
#include "spinning_joint_hardware_interface.hpp"

namespace romea
{

class HardwareInterface2WD
{
public:
  enum JointIDs
  {
    LEFT_WHEEL_SPINNING_JOINT_ID = 0,
    RIGHT_WHEEL_SPINNING_JOINT_ID = 1
  };

  HardwareInterface2WD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  HardwareCommand2WD get_command()const;
  void set_state(const HardwareState2WD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SpinningJointHardwareInterface left_wheel_spinning_joint_;
  SpinningJointHardwareInterface right_wheel_spinning_joint_;
};


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2WD_HPP_
