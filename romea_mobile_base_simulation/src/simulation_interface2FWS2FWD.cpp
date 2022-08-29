#include "romea_mobile_base_simulation/simulation_interface2FWS2FWD.hpp"
#include <romea_mobile_base_hardware/hardware_info.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
SimulationInterface2FWS2FWD::SimulationInterface2FWS2FWD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type):
  hardware_interface_(hardware_info,spinning_joint_command_interface_type),
  wheelbase_(get_parameter<double>(hardware_info,"wheelbase")),
  front_track_(get_parameter<double>(hardware_info,"front_track")),
  rear_track_(get_parameter<double>(hardware_info,"rear_track")),
  front_wheel_radius_(get_parameter<double>(hardware_info,"front_wheel_radius")),
  rear_wheel_radius_(get_parameter<double>(hardware_info,"rear_wheel_radius")),
  front_hub_carrier_offset_(get_parameter<double>(hardware_info,"front_hub_carrier_offset")),
  rear_hub_carrier_offset_(get_parameter<double>(hardware_info,"rear_hub_carrier_offset"))
{

}

//-----------------------------------------------------------------------------
SimulationCommand2FWS2FWD SimulationInterface2FWS2FWD::get_command()const
{

  return toSimulationCommand2FWS2FWD(wheelbase_,
                                     front_track_,
                                     rear_track_,
                                     front_wheel_radius_,
                                     rear_wheel_radius_,
                                     front_hub_carrier_offset_,
                                     rear_hub_carrier_offset_,
                                     hardware_interface_.get_command());

}

//-----------------------------------------------------------------------------
void SimulationInterface2FWS2FWD::set_state(const SimulationState2FWS2FWD & simulation_state)
{
  auto hardware_state =toHardwareState2FWS2FWD(simulation_state);

  hardware_interface_.set_state(hardware_state,
                                simulation_state.rearLeftWheelSpinMotion,
                                simulation_state.rearRightWheelSpinMotion);

}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2FWS2FWD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2FWS2FWD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}


}

