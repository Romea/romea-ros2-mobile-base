// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_GAZEBO__SPINNING_JOINT_GAZEBO_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__SPINNING_JOINT_GAZEBO_INTERFACE_HPP_

// std
#include <string>

// romea
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"

// gazebo
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"


namespace romea
{

class SpinningJointGazeboInterface
{
public:
  SpinningJointGazeboInterface(
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & command_interface_type);

  void set_command(const double & command);
  RotationalMotionState get_state()const;

private:
  RotationalMotionControlType control_type;
  gazebo::physics::JointPtr sim_joint_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__SPINNING_JOINT_GAZEBO_INTERFACE_HPP_
