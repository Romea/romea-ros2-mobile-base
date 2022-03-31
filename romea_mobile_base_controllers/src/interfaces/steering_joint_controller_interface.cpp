#include "romea_mobile_base_controllers/interfaces/steering_joint_controller_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SteeringJointControllerInterface::SteeringJointControllerInterface(const std::string & joint_name):
  JointControllerInterface (joint_name,hardware_interface::HW_IF_POSITION)
{
}

//-----------------------------------------------------------------------------
void SteeringJointControllerInterface::set_command(const double & command)
{
  assert(command_handle_);
  command_handle_->set_value(command);
}

//-----------------------------------------------------------------------------
double SteeringJointControllerInterface::get_measurement()const
{
  assert(state_handle_);
  return state_handle_->get_value();
}


}
