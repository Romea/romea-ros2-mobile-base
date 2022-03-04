#ifndef _romea_BaseHardwareInterface2FWS2FWD_hpp_
#define _romea_BaseHardwareInterface2FWS2FWD_hpp_

//romea
#include "spinning_joint_hardware_interface.hpp"
#include "steering_joint_hardware_interface.hpp"

namespace romea
{

struct HardwareInterface2FWS2FWD
{

  HardwareInterface2FWS2FWD(const hardware_interface::HardwareInfo & hardware_info,
                            const std::string & spinning_joint_command_interface_type);

  SteeringJointHardwareInterface front_left_wheel_steering_joint;
  SteeringJointHardwareInterface front_right_wheel_steering_joint;
  SpinningJointHardwareInterface front_left_wheel_spinning_joint;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint;

  SpinningJointHardwareInterface::Feedback rear_left_wheel_spinning_joint_feedback;
  SpinningJointHardwareInterface::Feedback rear_right_wheel_spinning_joint_feedback;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};


}

#endif
