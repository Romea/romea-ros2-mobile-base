#ifndef _romea_HardwareInterface1FAS4WD_hpp_
#define _romea_HardwareInterface1FAS4WD_hpp_

//romea
#include "spinning_joint_hardware_interface.hpp"
#include "steering_joint_hardware_interface.hpp"
#include <romea_core_mobile_base/hardware/HardwareControl1FAS4WD.hpp>

namespace romea
{

class HardwareInterface1FAS4WD
{

public :

  enum JointIds  {
    FRONT_AXLE_STEERING_JOINT_ID=0,
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID=1,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID=2,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID=3,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID=4,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID=5,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=6
  };

  HardwareInterface1FAS4WD(const hardware_interface::HardwareInfo & hardware_info,
                            const std::string & spinning_joint_command_interface_type);


  HardwareCommand1FAS4WD get_command()const;

  void set_state(const HardwareState1FAS4WD & hardware_state);

  void set_state(const HardwareState1FAS4WD & hardware_state,
                 const SteeringAngleState & front_left_wheel_steering_angle,
                 const SteeringAngleState & front_right_wheel_steering_angle);



  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:

  SteeringJointHardwareInterface front_axle_steering_joint_;
  SpinningJointHardwareInterface front_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;

  SteeringJointHardwareInterface::Feedback front_left_wheel_steering_joint_feedback_;
  SteeringJointHardwareInterface::Feedback front_right_wheel_steering_joint_feedback_;

};

}

#endif
