#ifndef _romea_SimulationInterface2AS4WD_hpp_
#define _romea_SimulationInterface2AS4WD_hpp_

//romea
#include <romea_mobile_base_hardware/hardware_interface2AS4WD.hpp>
#include <romea_core_mobile_base/simulation/SimulationControl2AS4WD.hpp>

namespace romea
{

class SimulationInterface2AS4WD
{

public :

  SimulationInterface2AS4WD(const hardware_interface::HardwareInfo & hardware_info,
                            const std::string & spinning_joint_command_interface_type);

  SimulationCommand2AS4WD get_command()const;
  void set_state(const SimulationState2AS4WD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private :

  HardwareInterface2AS4WD hardware_interface_;

  const double wheelbase_;
  const double front_track_;
  const double rear_track_;

};


}

#endif
