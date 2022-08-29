#include "romea_mobile_base_simulation/simulation_interface1FWS2RWD.hpp"
#include <romea_mobile_base_hardware/hardware_info.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
SimulationInterface1FWS2RWD::SimulationInterface1FWS2RWD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type):
  hardware_interface_(hardware_info,spinning_joint_command_interface_type),
  front_wheel_radius_(get_parameter<double>(hardware_info,"front_wheel_radius")),
  rear_wheel_radius_(get_parameter<double>(hardware_info,"rear_wheel_radius"))
{

}


//-----------------------------------------------------------------------------
SimulationCommand1FWS2RWD SimulationInterface1FWS2RWD::get_command()const
{
  return toSimulationCommand1FWS2RWD(front_wheel_radius_,
                                     rear_wheel_radius_,
                                     hardware_interface_.get_command());

}

//-----------------------------------------------------------------------------
void SimulationInterface1FWS2RWD::set_state(const SimulationState1FWS2RWD & simulation_state)
{
  auto hardware_state = toHardwareState1FWS2RWD(simulation_state);

  hardware_interface_.set_state(hardware_state,
                                simulation_state.frontWheelSpinMotion);

}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
SimulationInterface1FWS2RWD::export_state_interfaces()
{
  return hardware_interface_.export_state_interfaces();
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
SimulationInterface1FWS2RWD::export_command_interfaces()
{
  return hardware_interface_.export_command_interfaces();
}

}

