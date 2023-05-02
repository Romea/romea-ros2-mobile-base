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


// std
#include <vector>

// local
#include "romea_mobile_base_hardware/steering_joint_hardware_interface.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SteeringJointHardwareInterface::SteeringJointHardwareInterface(
  const hardware_interface::ComponentInfo & joint_info)
: command_(joint_info, hardware_interface::HW_IF_POSITION),
  feedback_(joint_info, hardware_interface::HW_IF_POSITION)
{
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::export_command_interface(
  std::vector<hardware_interface::CommandInterface> & command_interfaces)
{
  command_.export_interface(command_interfaces);
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::export_state_interface(
  std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  feedback_.export_interface(state_interfaces);
}

//-----------------------------------------------------------------------------
SteeringAngleCommand SteeringJointHardwareInterface::get_command()const
{
  return command_.get();
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::set_state(const SteeringAngleState & state)
{
  feedback_.set(state);
}

}  // namespace romea
