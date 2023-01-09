// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE2ASXXX_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE2ASXXX_HPP_

// romea
#include <romea_core_mobile_base/simulation/SimulationControl2ASxxx.hpp>

// std
#include <memory>
#include <string>
#include <vector>

// local
#include "romea_mobile_base_gazebo/spinning_joint_gazebo_interface.hpp"
#include "romea_mobile_base_gazebo/steering_joint_gazebo_interface.hpp"

namespace romea
{

class GazeboInterface2ASxxx
{
public:
  GazeboInterface2ASxxx(
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  SimulationState2ASxxx get_state() const;
  void set_command(const SimulationCommand2ASxxx & command);

private:
  SteeringJointGazeboInterface front_axle_steering_joint_;
  SteeringJointGazeboInterface rear_axle_steering_joint_;
  SteeringJointGazeboInterface front_left_wheel_steering_joint_;
  SteeringJointGazeboInterface front_right_wheel_steering_joint_;
  SteeringJointGazeboInterface rear_left_wheel_steering_joint_;
  SteeringJointGazeboInterface rear_right_wheel_steering_joint_;
  SpinningJointGazeboInterface front_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE2ASXXX_HPP_
