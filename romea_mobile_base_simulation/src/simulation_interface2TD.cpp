#include "romea_mobile_base_simulation/simulation_interface2TD.hpp"
#include <romea_mobile_base_hardware/hardware_info.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
SimulationInterface2TD::SimulationInterface2TD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type):
  hardware_interface_(hardware_info,command_interface_type),
  sprocket_wheel_radius_(get_parameter<double>(hardware_info,"sprocket_wheel_radius")),
  idler_wheel_radius_(get_parameter<double>(hardware_info,"idler_wheel_radius")),
  track_width_(get_parameter<double>(hardware_info,"track_width"))

{
}

//-----------------------------------------------------------------------------
SimulationCommand2TD SimulationInterface2TD::get_command()const
{
  return toSimulationCommand2TD(sprocket_wheel_radius_,
                                idler_wheel_radius_,
                                track_width_,
                                hardware_interface_.get_command());

}

//-----------------------------------------------------------------------------
void SimulationInterface2TD::set_state(const SimulationState2TD & simulation_state)
{
  auto hardware_state = toHardwareState2TD(sprocket_wheel_radius_,
                                           idler_wheel_radius_,
                                           track_width_,
                                           simulation_state);

  hardware_interface_.set_state(hardware_state,
                                simulation_state.leftIdlerWheelSpinMotion,
                                simulation_state.rightIdlerWheelSpinMotion);
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2TD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2TD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}



}

