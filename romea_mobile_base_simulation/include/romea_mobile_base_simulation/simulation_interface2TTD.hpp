#ifndef _romea_SimulationInterface2TTD_hpp_
#define _romea_SimulationInterface2TTD_hpp_

//romea
#include <romea_mobile_base_hardware/hardware_interface2TTD.hpp>
#include <romea_core_mobile_base/simulation/SimulationControl2TTD.hpp>

namespace romea
{

class SimulationInterface2TTD{

public :

  SimulationInterface2TTD(const hardware_interface::HardwareInfo & hardware_info,
                          const std::string & command_interface_type);

  SimulationCommand2TTD get_command()const;
  void set_state(const SimulationState2TTD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();


private :

  HardwareInterface2TTD hardware_interface_;

  const double idler_wheel_radius_;
  const double roller_wheel_radius_;
  const double sprocket_wheel_radius_;

};




}

#endif
