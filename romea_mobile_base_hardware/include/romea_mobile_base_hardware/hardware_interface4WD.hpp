// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WD_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/hardware/HardwareControl4WD.hpp"

// local
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"

namespace romea
{

class HardwareInterface4WD
{
public:
  enum JointIDs
  {
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID = 0,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID = 1,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID = 2,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID = 3
  };

  HardwareInterface4WD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  HardwareCommand4WD get_command() const;
  void set_state(const HardwareState4WD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SpinningJointHardwareInterface front_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface front_right_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_left_wheel_spinning_joint_;
  SpinningJointHardwareInterface rear_right_wheel_spinning_joint_;
};


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE4WD_HPP_
