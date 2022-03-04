#ifndef _romea_BaseHardwareInterface1FWS2RWD_hpp_
#define _romea_BaseHardwareInterface1FWS2RWD_hpp_

//romea
#include "spinning_joint_hardware_interface.hpp"
#include "steering_joint_hardware_interface.hpp"

namespace romea
{

struct HardwareInterface1FWS2RWD
{

  HardwareInterface1FWS2RWD(const hardware_interface::HardwareInfo & hardware_info,
                            const std::string & spinning_joint_command_interface_type);

  SteeringJointHardwareInterface front_wheel_steering_joint;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint;

  SpinningJointHardwareInterface::Feedback front_wheel_spinning_joint_feedback;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};



}

#endif