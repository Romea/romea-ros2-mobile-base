#ifndef _romea_SimulationInterface2THD_hpp_
#define _romea_SimulationInterface2THD_hpp_

//romea
#include <romea_mobile_base_hardware/hardware_interface2THD.hpp>
#include <romea_core_mobile_base/simulation/SimulationControl2THD.hpp>

namespace romea
{

class SimulationInterface2THD{

public :


  SimulationInterface2THD(const hardware_interface::HardwareInfo & hardware_info,
                          const std::string & command_interface_type);


  SimulationCommand2THD get_command()const;
  void set_state(const SimulationState2THD & simulation_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private :

  HardwareInterface2THD hardware_interface_;

  const double idler_wheel_radius_;
  const double sprocket_wheel_radius_;

};




}

#endif
