#ifndef _romea_BaseHardwareInterface1FWS2RWD_hpp_
#define _romea_BaseHardwareInterface1FWS2RWD_hpp_

//romea
#include "spinning_joint_hardware_interface.hpp"
#include "steering_joint_hardware_interface.hpp"
#include <romea_core_mobile_base/hardware/HardwareControl1FWS2RWD.hpp>


namespace romea
{

class HardwareInterface1FWS2RWD
{

public :

  enum JointIDs  {
    FRONT_WHEEL_STEERING_JOINT_ID=0,
    FRONT_WHEEL_SPINNING_JOINT_ID=1,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID=2,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=3
  };

  HardwareInterface1FWS2RWD(const hardware_interface::HardwareInfo & hardware_info,
                            const std::string & spinning_joint_command_interface_type);


  HardwareCommand1FWS2RWD get_command()const;

  void set_state(const HardwareState1FWS2RWD & hardware_state);

  void set_state(const HardwareState1FWS2RWD & hardware_state,
                 const RotationalMotionState & front_wheel_set_point);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private :

  SteeringJointHardwareInterface front_wheel_steering_joint_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;

  SpinningJointHardwareInterface::Feedback front_wheel_spinning_joint_feedback_;

};




}

#endif
