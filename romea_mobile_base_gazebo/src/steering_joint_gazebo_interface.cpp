// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#include "romea_mobile_base_gazebo/steering_joint_gazebo_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SteeringJointGazeboInterface::SteeringJointGazeboInterface(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::ComponentInfo & joint_info)
{
  std::cout << " steering joint_info.name ";
  std::cout << joint_info.name << std::endl;

  sim_joint_ = parent_model->GetJoint(joint_info.name);

  std::cout << " steering joint_info.name ";
  std::cout << joint_info.name << " ";
  std::cout << int(sim_joint_.get() != nullptr) << std::endl;
}

//-----------------------------------------------------------------------------
void SteeringJointGazeboInterface::set_command(const double & command)
{
  sim_joint_->SetPosition(0, command, true);
}

//-----------------------------------------------------------------------------
double SteeringJointGazeboInterface::get_state() const
{
  return sim_joint_->Position(0);
}

}  // namespace romea
