#ifndef _romea_HardwareInterface1FAS2RWD_hpp_
#define _romea_HardwareInterface1FAS2RWD_hpp_

//romea
#include "hardware_spinning_joint_interface.hpp"
#include "hardware_steering_joint_interface.hpp"

namespace romea
{

struct HardwareInterface1FAS2RWD
{

  HardwareInterface1FAS2RWD(const hardware_interface::HardwareInfo & hardware_info,
                            const std::string & spinning_joint_command_interface_type);

  HardwareSteeringJointInterface front_axle_steering_joint;
  HardwareSpinningJointInterface rear_left_wheel_spinning_joint;
  HardwareSpinningJointInterface rear_right_wheel_spinning_joint;

  HardwareSteeringJointInterface::Feedback front_left_wheel_steering_joint_feedback;
  HardwareSteeringJointInterface::Feedback front_right_wheel_steering_joint_feedback;
  HardwareSpinningJointInterface::Feedback front_left_wheel_spinning_joint_feedback;
  HardwareSpinningJointInterface::Feedback front_right_wheel_spinning_joint_feedback;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};



}

#endif
