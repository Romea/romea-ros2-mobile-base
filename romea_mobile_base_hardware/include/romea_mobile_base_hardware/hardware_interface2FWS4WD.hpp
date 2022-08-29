#ifndef _romea_HardwareInterface2FWS4WD_hpp_
#define _romea_HardwareInterface2FWS4WD_hpp_

#include "spinning_joint_hardware_interface.hpp"
#include "steering_joint_hardware_interface.hpp"
#include <romea_core_mobile_base/hardware/HardwareControl2FWS4WD.hpp>

namespace romea
{

class HardwareInterface2FWS4WD
{

public:

  enum JointIDs {
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID=0,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID=1,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID=2,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID=3,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID=4,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=5
  };

  HardwareInterface2FWS4WD(const hardware_interface::HardwareInfo & hardware_info,
                           const std::string & wheel_spinning_joint_command_interface_type);

  HardwareCommand2FWS4WD get_command()const;
  void set_state(const HardwareState2FWS4WD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private :

  SteeringJointHardwareInterface front_left_wheel_steering_joint_;
  SteeringJointHardwareInterface front_right_wheel_steering_joint_;
  SpinningJointHardwareInterface front_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;

};




}

#endif
