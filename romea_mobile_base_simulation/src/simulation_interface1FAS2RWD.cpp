// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// romea
#include <romea_mobile_base_hardware/hardware_info.hpp>

// std
#include <string>
#include <vector>

// local
#include "romea_mobile_base_simulation/simulation_interface1FAS2RWD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SimulationInterface1FAS2RWD::SimulationInterface1FAS2RWD(
  const hardware_interface::HardwareInfo & hardware_info,
  const std::string & spinning_joint_command_interface_type)
: hardware_interface_(hardware_info, spinning_joint_command_interface_type),
  wheelbase_(get_parameter<double>(hardware_info, "wheelbase")),
  front_track_(get_parameter<double>(hardware_info, "front_track")),
  front_wheel_radius_(get_parameter<double>(hardware_info, "front_wheel_radius")),
  front_hub_carrier_offset_(get_parameter<double>(hardware_info, "front_hub_carrier_offset")),
  rear_wheel_radius_(get_parameter<double>(hardware_info, "rear_wheel_radius"))
{
}


//-----------------------------------------------------------------------------
SimulationCommand1FAS2RWD SimulationInterface1FAS2RWD::get_command()const
{
  return toSimulationCommand1FAS2RWD(
    wheelbase_,
    front_track_,
    front_hub_carrier_offset_,
    front_wheel_radius_,
    rear_wheel_radius_,
    hardware_interface_.get_command());
}

//-----------------------------------------------------------------------------
void SimulationInterface1FAS2RWD::set_state(const SimulationState1FAS2RWD & simulation_state)
{
  auto hardware_state = toHardwareState1FAS2RWD(
    wheelbase_,
    front_track_,
    simulation_state);

  hardware_interface_.set_state(
    hardware_state,
    simulation_state.frontLeftWheelSteeringAngle,
    simulation_state.frontRightWheelSteeringAngle,
    simulation_state.frontLeftWheelSpinningMotion,
    simulation_state.frontRightWheelSpinningMotion);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface1FAS2RWD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface1FAS2RWD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}

}  // namespace romea
