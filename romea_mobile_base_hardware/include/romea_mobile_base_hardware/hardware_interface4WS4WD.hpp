// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WS4WD_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WS4WD_HPP_

// romea
#include <romea_core_mobile_base/hardware/HardwareControl4WS4WD.hpp>

// std
#include <string>
#include <vector>

// local
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"
#include "romea_mobile_base_hardware/steering_joint_hardware_interface.hpp"


namespace romea
{

class HardwareInterface4WS4WD
{
public:
  enum JointIDs
  {
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID = 0,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID = 1,
    REAR_LEFT_WHEEL_STEERING_JOINT_ID = 2,
    REAR_RIGHT_WHEEL_STEERING_JOINT_ID = 3,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID = 4,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID = 5,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID = 6,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID = 7
  };

  HardwareInterface4WS4WD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type);

  HardwareCommand4WS4WD get_command()const;
  void set_state(const HardwareState4WS4WD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SteeringJointHardwareInterface front_left_wheel_steering_joint_;
  SteeringJointHardwareInterface front_right_wheel_steering_joint_;
  SteeringJointHardwareInterface rear_left_wheel_steering_joint_;
  SteeringJointHardwareInterface rear_right_wheel_steering_joint_;
  SpinningJointHardwareInterface front_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WS4WD_HPP_
