#ifndef _romea_SimulationInterface2TD_hpp_
#define _romea_SimulationInterface2TD_hpp_

//romea
#include <romea_mobile_base_hardware/hardware_interface2TD.hpp>
#include <romea_core_mobile_base/simulation/SimulationControl2TD.hpp>


namespace romea
{

class SimulationInterface2TD{

public:

  SimulationInterface2TD(const hardware_interface::HardwareInfo & hardware_info,
                         const std::string & command_interface_type);

  SimulationCommand2TD get_command()const;
  void set_state(const SimulationState2TD & simulation_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private :

  HardwareInterface2TD hardware_interface_;

  const double sprocket_wheel_radius_;
  const double idler_wheel_radius_;
  const double track_thickness_;
};




}

#endif
