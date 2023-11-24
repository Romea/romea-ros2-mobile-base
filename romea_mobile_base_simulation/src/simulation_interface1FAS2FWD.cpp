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
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_hardware/hardware_info.hpp"

// local
#include "romea_mobile_base_simulation/simulation_interface1FAS2FWD.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface1FAS2FWD::SimulationInterface1FAS2FWD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: hardware_interface_(hardware_info, spinning_joint_command_interface_type),
  wheelbase_(get_parameter<double>(hardware_info, "wheelbase")),
  front_track_(get_parameter<double>(hardware_info, "front_track")),
  front_wheel_radius_(get_parameter<double>(hardware_info, "front_wheel_radius")),
  front_hub_carrier_offset_(get_parameter<double>(hardware_info, "front_hub_carrier_offset")),
  rear_track_(get_parameter<double>(hardware_info, "rear_track")),
  rear_wheel_radius_(get_parameter<double>(hardware_info, "rear_wheel_radius")),
  rear_hub_carrier_offset_(get_parameter<double>(hardware_info, "rear_hub_carrier_offset"))
{
}


//-----------------------------------------------------------------------------
core::SimulationCommand1FAS2FWD SimulationInterface1FAS2FWD::get_command()const
{
  return toSimulationCommand1FAS2FWD(
    wheelbase_,
    front_track_,
    rear_track_,
    front_wheel_radius_,
    rear_wheel_radius_,
    front_hub_carrier_offset_,
    rear_hub_carrier_offset_,
    hardware_interface_.get_command());
}

//-----------------------------------------------------------------------------
void SimulationInterface1FAS2FWD::set_state(const core::SimulationState1FAS2FWD & simulation_state)
{
  auto hardware_state = toHardwareState1FAS2FWD(
    wheelbase_,
    front_track_,
    simulation_state);

  hardware_interface_.set_state(
    hardware_state,
    simulation_state.frontLeftWheelSteeringAngle,
    simulation_state.frontRightWheelSteeringAngle,
    simulation_state.rearLeftWheelSpinningMotion,
    simulation_state.rearRightWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface1FAS2FWD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface1FAS2FWD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}

}  // namespace ros2
}  // namespace romea
