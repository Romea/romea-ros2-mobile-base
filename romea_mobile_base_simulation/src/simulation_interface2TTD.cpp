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

// local
#include "romea_mobile_base_simulation/simulation_interface2TTD.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SimulationInterface2TTD::SimulationInterface2TTD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & command_interface_type)
: hardware_interface_(hardware_info, command_interface_type),
  idler_wheel_radius_(get_parameter<double>(hardware_info, "idler_wheel_radius")),
  roller_wheel_radius_(get_parameter<double>(hardware_info, "roller_wheel_radius")),
  sprocket_wheel_radius_(get_parameter<double>(hardware_info, "sprocket_wheel_radius")),
  track_thickness_(get_parameter<double>(hardware_info, "track_thickness"))
{
}

//-----------------------------------------------------------------------------
core::SimulationCommand2TTD SimulationInterface2TTD::get_command()const
{
  return toSimulationCommand2TTD(
    sprocket_wheel_radius_,
    idler_wheel_radius_,
    roller_wheel_radius_,
    track_thickness_,
    hardware_interface_.get_command());
}

//-----------------------------------------------------------------------------
void SimulationInterface2TTD::set_state(const core::SimulationState2TTD & simulation_state)
{
  auto hardware_state = toHardwareState2TTD(
    sprocket_wheel_radius_,
    roller_wheel_radius_,
    track_thickness_,
    simulation_state);

  hardware_interface_.set_state(
    hardware_state,
    simulation_state.leftIdlerWheelSpinningMotion,
    simulation_state.rightIdlerWheelSpinningMotion,
    simulation_state.frontLeftRollerWheelSpinningMotion,
    simulation_state.frontRightRollerWheelSpinningMotion,
    simulation_state.rearLeftRollerWheelSpinningMotion,
    simulation_state.rearRightRollerWheelSpinningMotion);
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2TTD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2TTD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}

}  // namespace ros2
}  // namespace romea
