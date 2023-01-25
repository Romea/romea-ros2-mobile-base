// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE2THD_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE2THD_HPP_

// std
#include <string>

// romea
#include "romea_core_mobile_base/simulation/SimulationControl2THD.hpp"

// local
#include "romea_mobile_base_gazebo/spinning_joint_gazebo_interface.hpp"


namespace romea
{

class GazeboInterface2THD
{
public:
  GazeboInterface2THD(
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  SimulationState2THD get_state() const;
  void set_command(const SimulationCommand2THD & command);

private:
  SpinningJointGazeboInterface left_sprocket_wheel_spinning_joint_;
  SpinningJointGazeboInterface right_sprocket_wheel_spinning_joint_;
  SpinningJointGazeboInterface front_left_idler_wheel_spinning_joint_;
  SpinningJointGazeboInterface front_right_idler_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_left_idler_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_right_idler_wheel_spinning_joint_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_INTERFACE2THD_HPP_
