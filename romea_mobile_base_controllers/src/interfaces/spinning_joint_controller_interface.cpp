#include "romea_mobile_base_controllers/interfaces/spinning_joint_controller_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SpinningJointControllerInterface::SpinningJointControllerInterface(const std::string & joint_name,
                                                                   const double &wheel_radius):
  JointControllerInterface (joint_name,hardware_interface::HW_IF_VELOCITY),
  wheel_radius_(wheel_radius)
{
}

//-----------------------------------------------------------------------------
void SpinningJointControllerInterface::set_command(const double & command)
{
  assert(command_handle_);
  command_handle_->set_value(command/wheel_radius_);
}


//-----------------------------------------------------------------------------
double SpinningJointControllerInterface::get_measurement()const
{
  assert(state_handle_);
  return state_handle_->get_value()*wheel_radius_;
}

}
