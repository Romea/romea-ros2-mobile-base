#ifndef _romea_HardwareInterface2WD_hpp_
#define _romea_HardwareInterface2WD_hpp_

#include "spinning_joint_hardware_interface.hpp"
#include <romea_core_mobile_base/hardware/HardwareControl2WD.hpp>

namespace romea
{

class HardwareInterface2WD{

public:

  enum JointIDs  {
    LEFT_WHEEL_SPINNING_JOINT_ID=0,
    RIGHT_WHEEL_SPINNING_JOINT_ID=1
  };

  HardwareInterface2WD(const hardware_interface::HardwareInfo & hardware_info,
                       const std::string & command_interface_type);

  HardwareCommand2WD get_command()const;
  void set_state(const HardwareState2WD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private :

  SpinningJointHardwareInterface left_wheel_spinning_joint_;
  SpinningJointHardwareInterface right_wheel_spinning_joint_;

};


}

#endif
