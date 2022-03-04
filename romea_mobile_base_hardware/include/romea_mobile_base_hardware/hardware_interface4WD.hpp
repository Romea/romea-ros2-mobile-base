#ifndef _romea_HardwareInterface4WD_hpp_
#define _romea_HardwareInterface4WD_hpp_

#include "spinning_joint_hardware_interface.hpp"

namespace romea
{

struct HardwareInterface4WD{

  HardwareInterface4WD(const hardware_interface::HardwareInfo & hardware_info,
                       const std::string & command_interface_type);

  SpinningJointHardwareInterface front_left_wheel_spinning_joint;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};


}

#endif
