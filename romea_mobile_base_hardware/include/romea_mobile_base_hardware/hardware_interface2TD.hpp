#ifndef _romea_HardwareInterface2TD_hpp_
#define _romea_HardwareInterface2TD_hpp_

#include "spinning_joint_hardware_interface.hpp"

namespace romea
{

struct HardwareInterface2TD{

  enum JointIDs  {
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID=0,
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID=1,
    LEFT_IDLER_WHEEL_SPINNING_JOINT_ID=2,
    RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID=3
  };

  HardwareInterface2TD(const hardware_interface::HardwareInfo & hardware_info,
                       const std::string & command_interface_type);

  SpinningJointHardwareInterface left_sprocket_wheel_spinning_joint;
  SpinningJointHardwareInterface right_sprocket_wheel_spinning_joint;
  SpinningJointHardwareInterface left_idler_wheel_spinning_joint;
  SpinningJointHardwareInterface right_idler_wheel_spinning_joint;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};

}

#endif
