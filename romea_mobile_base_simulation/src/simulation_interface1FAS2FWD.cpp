// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_mobile_base_hardware/hardware_info.hpp>

// std
#include <string>
#include <vector>

// local
#include "romea_mobile_base_simulation/simulation_interface1FAS2FWD.hpp"

namespace romea
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
SimulationCommand1FAS2FWD SimulationInterface1FAS2FWD::get_command()const
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
void SimulationInterface1FAS2FWD::set_state(const SimulationState1FAS2FWD & simulation_state)
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

}  // namespace romea
