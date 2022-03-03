#ifndef _romea_BaseHardwareInterface2FWS2FWD_hpp_
#define _romea_BaseHardwareInterface2FWS2FWD_hpp_

//romea
#include "hardware_spinning_joint_interface.hpp"
#include "hardware_steering_joint_interface.hpp"

namespace romea
{

struct HardwareInterface2FWS2FWD
{

  HardwareInterface2FWS2FWD(const hardware_interface::HardwareInfo & hardware_info,
                            const std::string & spinning_joint_command_interface_type);

  HardwareSteeringJointInterface front_left_wheel_steering_joint;
  HardwareSteeringJointInterface front_right_wheel_steering_joint;
  HardwareSpinningJointInterface front_left_wheel_spinning_joint;
  HardwareSpinningJointInterface front_right_wheel_spinning_joint;

  HardwareSpinningJointInterface::Feedback rear_left_wheel_spinning_joint_feedback;
  HardwareSpinningJointInterface::Feedback rear_right_wheel_spinning_joint_feedback;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};


}

#endif
