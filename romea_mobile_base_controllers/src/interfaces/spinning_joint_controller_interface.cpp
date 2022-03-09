#include "romea_mobile_base_controllers/interfaces/spinning_joint_controller_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SpinningJointControllerInterface::SpinningJointControllerInterface(LoanedCommandInterfaces &loaned_command_interfaces,
                                                                   LoanedStateInterfaces &loaned_state_interfaces,
                                                                   const std::string & joint_name, const double &wheel_radius):
  JointControllerInterface (loaned_command_interfaces,
                            loaned_state_interfaces,
                            hardware_interface::HW_IF_VELOCITY,
                            joint_name),
  wheel_radius_(wheel_radius)
{
}

//-----------------------------------------------------------------------------
void SpinningJointControllerInterface::setCommand(const double & command)
{
    command_handle_.get().set_value(command/wheel_radius_);
}


//-----------------------------------------------------------------------------
double SpinningJointControllerInterface::getMeasurement()const
{
    return  state_handle_.get().get_value()*wheel_radius_;
}

}
