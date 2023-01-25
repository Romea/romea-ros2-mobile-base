// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE1FAS2FWD_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE1FAS2FWD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/hardware/HardwareControl1FAS2FWD.hpp"

// local
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"
#include "romea_mobile_base_hardware/steering_joint_hardware_interface.hpp"

namespace romea
{

class HardwareInterface1FAS2FWD
{
public:
  enum JointIds
  {
    FRONT_AXLE_STEERING_JOINT_ID = 0,
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID = 1,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID = 2,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID = 3,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID = 4,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID = 5,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID = 6
  };

  HardwareInterface1FAS2FWD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type);

  HardwareCommand1FAS2FWD get_command()const;

  void set_state(const HardwareState1FAS2FWD & hardware_state);

  void set_state(
    const HardwareState1FAS2FWD & hardware_state,
    const SteeringAngleState & front_left_wheel_steering_angle,
    const SteeringAngleState & front_right_wheel_steering_angle,
    const RotationalMotionState & rear_left_wheel_motion_state,
    const RotationalMotionState & rear_right_wheel_motion_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SteeringJointHardwareInterface front_axle_steering_joint_;
  SpinningJointHardwareInterface front_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint_;

  SteeringJointHardwareInterface::Feedback front_left_wheel_steering_joint_feedback_;
  SteeringJointHardwareInterface::Feedback front_right_wheel_steering_joint_feedback_;
  SpinningJointHardwareInterface::Feedback rear_left_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback rear_right_wheel_spinning_joint_feedback_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE1FAS2FWD_HPP_
