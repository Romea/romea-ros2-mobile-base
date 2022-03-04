#ifndef _romea_HardwareSteeringJointInterface_hpp_
#define _romea_HardwareSteeringJointInterface_hpp_

#include "hardware_handle.hpp"

namespace romea
{

struct SteeringJointHardwareInterface
{
  using Command = HardwareCommandInterface;
  using Feedback = HardwareStateInterface;

  SteeringJointHardwareInterface(const hardware_interface::ComponentInfo &joint_info);

  Command command;
  Feedback feedback;

  void export_command_interface(std::vector<hardware_interface::CommandInterface> & hardware_interfaces);
  void export_state_interface(std::vector<hardware_interface::StateInterface> & hardware_interfaces);

};

}
#endif
