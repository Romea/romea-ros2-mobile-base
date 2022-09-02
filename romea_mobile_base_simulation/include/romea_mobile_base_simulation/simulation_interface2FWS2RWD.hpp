#ifndef _romea_SimulationInterface2FWS2RWD_hpp_
#define _romea_SimulationInterface2FWS2RWD_hpp_


//romea
#include <romea_mobile_base_hardware/hardware_interface2FWS2RWD.hpp>
#include <romea_core_mobile_base/simulation/SimulationControl2FWS2RWD.hpp>


namespace romea
{

class SimulationInterface2FWS2RWD
{

public:

  SimulationInterface2FWS2RWD(const hardware_interface::HardwareInfo & hardware_info,
                              const std::string & spinning_joint_command_interface_type);

  SimulationCommand2FWS2RWD get_command()const;
  void set_state(const SimulationState2FWS2RWD & hardware_state);


  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private :

  HardwareInterface2FWS2RWD hardware_interface_;

  const double wheelbase_;
  const double front_track_;
  const double front_hub_carrier_offset_;
  const double front_wheel_radius_;
  const double rear_wheel_radius_;

};



}

#endif