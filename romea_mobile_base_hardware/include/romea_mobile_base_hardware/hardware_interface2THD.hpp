#ifndef _romea_HardwareInterface2THD_hpp_
#define _romea_HardwareInterface2THD_hpp_

#include "spinning_joint_hardware_interface.hpp"

namespace romea
{

struct HardwareInterface2THD{

  enum JointIDs  {
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID=0,
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID=1,
    FRONT_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID=2,
    FRONT_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID=3,
    REAR_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID=4,
    REAR_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID=5

  };

  HardwareInterface2THD(const hardware_interface::HardwareInfo & hardware_info,
                        const std::string & command_interface_type);

  SpinningJointHardwareInterface left_sprocket_wheel_spinning_joint;
  SpinningJointHardwareInterface right_sprocket_wheel_spinning_joint;
  SpinningJointHardwareInterface::Feedback front_left_idler_wheel_spinning_joint_feedback;
  SpinningJointHardwareInterface::Feedback front_right_idler_wheel_spinning_joint_feedback;
  SpinningJointHardwareInterface::Feedback rear_left_idler_wheel_spinning_joint_feedback;
  SpinningJointHardwareInterface::Feedback rear_right_idler_wheel_spinning_joint_feedback;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();
};


}

#endif
