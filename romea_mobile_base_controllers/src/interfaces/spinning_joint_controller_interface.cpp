#include "romea_mobile_base_controllers/interfaces/spinning_joint_controller_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SpinningJointControllerInterface::SpinningJointControllerInterface(const double & wheel_radius):
  wheel_radius_(wheel_radius)
{
}

//-----------------------------------------------------------------------------
void SpinningJointControllerInterface::write(const double & command, LoanedCommandInterface & loaned_command_interface) const
{
  loaned_command_interface.set_value(command/wheel_radius_);
}

//-----------------------------------------------------------------------------
void SpinningJointControllerInterface::read(const LoanedStateInterface & loaned_state_interface, double & measurement)const
{
  measurement= loaned_state_interface.get_value()*wheel_radius_;
}

//-----------------------------------------------------------------------------
std::string SpinningJointControllerInterface::hardware_interface_name(const std::string & joint_name)
{
  return joint_name+"/"+hardware_interface::HW_IF_VELOCITY;
}

}
