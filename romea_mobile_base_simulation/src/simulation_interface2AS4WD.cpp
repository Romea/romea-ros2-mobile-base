// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_hardware/hardware_info.hpp"

// local
#include "romea_mobile_base_simulation/simulation_interface2AS4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SimulationInterface2AS4WD::SimulationInterface2AS4WD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: hardware_interface_(hardware_info, spinning_joint_command_interface_type),
  wheelbase_(get_parameter<double>(hardware_info, "wheelbase")),
  front_track_(get_parameter<double>(hardware_info, "front_track")),
  rear_track_(get_parameter<double>(hardware_info, "rear_track"))
{
}

//-----------------------------------------------------------------------------
SimulationCommand2AS4WD SimulationInterface2AS4WD::get_command()const
{
  return toSimulationCommand2AS4WD(
    wheelbase_,
    front_track_,
    rear_track_,
    hardware_interface_.get_command());
}

//-----------------------------------------------------------------------------
void SimulationInterface2AS4WD::set_state(const SimulationState2AS4WD & simulation_state)
{
  auto hardware_state = toHardwareState2AS4WD(
    wheelbase_,
    front_track_,
    rear_track_,
    simulation_state);

  hardware_interface_.set_state(
    hardware_state,
    simulation_state.frontLeftWheelSteeringAngle,
    simulation_state.frontRightWheelSteeringAngle,
    simulation_state.rearLeftWheelSteeringAngle,
    simulation_state.rearRightWheelSteeringAngle);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2AS4WD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2AS4WD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}

}  // namespace romea
