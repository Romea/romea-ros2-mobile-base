#ifndef _romea_HardwareInterface2WD_hpp_
#define _romea_HardwareInterface2WD_hpp_

#include "spinning_joint_hardware_interface.hpp"

namespace romea
{

struct HardwareInterface2WD{

  enum JointIDs  {
    LEFT_WHEEL_SPINNING_JOINT_ID=0,
    RIGHT_WHEEL_SPINNING_JOINT_ID=1
  };

  HardwareInterface2WD(const hardware_interface::HardwareInfo & hardware_info,
                       const std::string & command_interface_type);

  SpinningJointHardwareInterface left_wheel_spinning_joint;
  SpinningJointHardwareInterface right_wheel_spinning_joint;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};

}

#endif
