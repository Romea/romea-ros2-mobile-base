#include "romea_mobile_base_controllers/interfaces/steering_joint_controller_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SteeringJointControllerInterface::SteeringJointControllerInterface(LoanedCommandInterfaces &loaned_command_interfaces,
                                                                   LoanedStateInterfaces &loaned_state_interfaces,
                                                                   const std::string & joint_name):
  JointControllerInterface (loaned_command_interfaces,
                            loaned_state_interfaces,
                            hardware_interface::HW_IF_POSITION,
                            joint_name)
{

}

//-----------------------------------------------------------------------------
void SteeringJointControllerInterface::setCommand(const double & command)
{
  command_handle_.get().set_value(command);
}

//-----------------------------------------------------------------------------
double SteeringJointControllerInterface::getMeasurement()const
{
  return state_handle_.get().get_value();
}


}
