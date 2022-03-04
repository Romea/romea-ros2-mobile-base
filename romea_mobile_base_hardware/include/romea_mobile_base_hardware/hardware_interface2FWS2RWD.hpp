#ifndef _romea_HardwareInterface2FWS2RWD_hpp_
#define _romea_HardwareInterface2FWS2RWD_hpp_


//romea
#include "spinning_joint_hardware_interface.hpp"
#include "steering_joint_hardware_interface.hpp"


namespace romea
{

struct HardwareInterface2FWS2RWD
{

  enum JointIDs  {
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID=0,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID=1,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID=2,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID=3,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID=4,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=5
  };

  HardwareInterface2FWS2RWD(const hardware_interface::HardwareInfo & hardware_info,
                            const std::string & spinning_joint_command_interface_type);

  SteeringJointHardwareInterface front_left_wheel_steering_joint;
  SteeringJointHardwareInterface front_right_wheel_steering_joint;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint;

  SpinningJointHardwareInterface::Feedback front_left_wheel_spinning_joint_feedback;
  SpinningJointHardwareInterface::Feedback front_right_wheel_spinning_joint_feedback;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};

}

#endif
