// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2THD_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2THD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_core_mobile_base/hardware/HardwareControl2TD.hpp"

// local
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"


namespace romea
{
class HardwareInterface2THD
{
public:
  enum JointIDs
  {
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID = 0,
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID = 1,
    FRONT_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID = 2,
    FRONT_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID = 3,
    REAR_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID = 4,
    REAR_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID = 5
  };

  HardwareInterface2THD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);


  HardwareCommand2TD get_command()const;

  void set_state(const HardwareState2TD & hardware_state);

  void set_state(
    const HardwareState2TD & hardware_state,
    const RotationalMotionState & front_left_idler_wheel_spinning_set_point,
    const RotationalMotionState & front_right_idler_wheel_spinning_set_point,
    const RotationalMotionState & rear_left_idler_wheel_spinning_set_point,
    const RotationalMotionState & rear_right_idler_wheel_spinning_set_point);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  SpinningJointHardwareInterface left_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface right_sprocket_wheel_spinning_joint_;
  SpinningJointHardwareInterface::Feedback front_left_idler_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback front_right_idler_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback rear_left_idler_wheel_spinning_joint_feedback_;
  SpinningJointHardwareInterface::Feedback rear_right_idler_wheel_spinning_joint_feedback_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_INTERFACE2THD_HPP_
