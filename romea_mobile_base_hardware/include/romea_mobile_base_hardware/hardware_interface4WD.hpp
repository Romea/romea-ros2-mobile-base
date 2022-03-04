#ifndef _romea_HardwareInterface4WD_hpp_
#define _romea_HardwareInterface4WD_hpp_

#include "spinning_joint_hardware_interface.hpp"

namespace romea
{

struct HardwareInterface4WD{

  enum JointIDs{
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID=0,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID=1,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID=2,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=3
  };

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
