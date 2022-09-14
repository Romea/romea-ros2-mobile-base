#include "romea_mobile_base_simulation/simulation_interface2FWS2RWD.hpp"
#include <romea_mobile_base_hardware/hardware_info.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
SimulationInterface2FWS2RWD::SimulationInterface2FWS2RWD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type):
  hardware_interface_(hardware_info,spinning_joint_command_interface_type),
  wheelbase_(get_parameter<double>(hardware_info,"wheelbase")),
  front_track_(get_parameter<double>(hardware_info,"front_track")),
  front_hub_carrier_offset_(get_parameter<double>(hardware_info,"front_hub_carrier_offset")),
  front_wheel_radius_(get_parameter<double>(hardware_info,"front_wheel_radius")),
  rear_wheel_radius_(get_parameter<double>(hardware_info,"rear_wheel_radius"))
{

}

//-----------------------------------------------------------------------------
SimulationCommand2FWS2RWD SimulationInterface2FWS2RWD::get_command()const
{
  return toSimulationCommand2FWS2RWD(wheelbase_,
                                     front_track_,
                                     front_hub_carrier_offset_,
                                     front_wheel_radius_,
                                     rear_wheel_radius_,
                                     hardware_interface_.get_command());
}


//-----------------------------------------------------------------------------
void SimulationInterface2FWS2RWD::set_state(const SimulationState2FWS2RWD & simulation_state)
{
  auto hardware_state=toHardwareState2FWS2RWD(simulation_state);

  hardware_interface_.set_state(hardware_state,
                                simulation_state.frontLeftWheelSpinningMotion,
                                simulation_state.frontRightWheelSpinningMotion);

}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2FWS2RWD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2FWS2RWD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}


}

