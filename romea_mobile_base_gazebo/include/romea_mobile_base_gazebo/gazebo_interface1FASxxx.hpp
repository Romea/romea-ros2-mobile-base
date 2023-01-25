// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE1FASXXX_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE1FASXXX_HPP_

// std
#include <string>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl1FASxxx.hpp"

// local
#include "romea_mobile_base_gazebo/spinning_joint_gazebo_interface.hpp"
#include "romea_mobile_base_gazebo/steering_joint_gazebo_interface.hpp"

namespace romea
{

struct GazeboInterface1FASxxx
{
  GazeboInterface1FASxxx(
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  SimulationState1FASxxx get_state() const;
  void set_command(const SimulationCommand1FASxxx & command);

private:
  SteeringJointGazeboInterface front_axle_steering_joint_;
  SteeringJointGazeboInterface front_left_wheel_steering_joint_;
  SteeringJointGazeboInterface front_right_wheel_steering_joint_;
  SpinningJointGazeboInterface front_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE1FASXXX_HPP_
