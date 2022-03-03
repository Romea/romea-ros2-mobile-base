#ifndef _romea_HardwareSpinningJointInterface_hpp_
#define _romea_HardwareSpinningJointInterface_hpp_

#include "hardware_handle.hpp"

namespace romea
{

struct HardwareSpinningJointInterface
{
  using Command = HardwareCommandInterface;

  struct Feedback
  {
    Feedback(const hardware_interface::ComponentInfo & joint_info);
    HardwareStateInterface position;
    HardwareStateInterface velocity;
    HardwareStateInterface torque;
    void export_state_interfaces(std::vector<hardware_interface::StateInterface> & state_interfaces);
  };

  HardwareSpinningJointInterface(const hardware_interface::ComponentInfo & joint_info,
                                 const std::string & command_interface_type = hardware_interface::HW_IF_VELOCITY);

  Command command;
  Feedback feedback;

  void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces);
  void export_state_interfaces(std::vector<hardware_interface::StateInterface> & state_interfaces);

};





}
#endif
