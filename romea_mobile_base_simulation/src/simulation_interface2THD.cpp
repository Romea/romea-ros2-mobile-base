#include "romea_mobile_base_simulation/simulation_interface2THD.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
SimulationInterface2THD::SimulationInterface2THD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type):
  hardware_interface_(hardware_info,command_interface_type),
  idler_wheel_radius_(get_parameter<double>(hardware_info,"idler_wheel_radius")),
  sprocket_wheel_radius_(get_parameter<double>(hardware_info,"sprocket_wheel_radius"))

{
}

//-----------------------------------------------------------------------------
SimulationCommand2THD SimulationInterface2THD::get_command()const
{
  return toSimulationCommand2THD(sprocket_wheel_radius_,
                                 idler_wheel_radius_,
                                 hardware_interface_.get_command());
}

//-----------------------------------------------------------------------------
void SimulationInterface2THD::set_state(const SimulationState2THD & simulation_state)
{

  auto hardware_state =toHardwareState2TD(sprocket_wheel_radius_,
                                          idler_wheel_radius_,
                                          simulation_state);

  hardware_interface_.set_state(hardware_state,
                                simulation_state.frontLeftIdlerWheelSpinMotion,
                                simulation_state.frontRightIdlerWheelSpinMotion,
                                simulation_state.rearLeftIdlerWheelSpinMotion,
                                simulation_state.rearRightIdlerWheelSpinMotion);

}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface2THD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface2THD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}

}

