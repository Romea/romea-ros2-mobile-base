#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

namespace romea
{

RotationalMotionControlType toRotationalMotionCommandType(const std::string & interface_type)
{
  if(interface_type==hardware_interface::HW_IF_VELOCITY)
  {
    return RotationalMotionControlType::VELOCITY;
  }
  else if (interface_type==hardware_interface::HW_IF_EFFORT)
  {
    return RotationalMotionControlType::TORQUE;
  }
  else
  {

  }
}

//-----------------------------------------------------------------------------
SpinningJointHardwareInterface::SpinningJointHardwareInterface(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & spinning_joint_command_interface_type):
  command_(joint_info,spinning_joint_command_interface_type),
  feedback_(joint_info)
{

}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces)
{
  command_.export_interface(command_interfaces);
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::export_state_interfaces(
    std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  feedback_.export_state_interfaces(state_interfaces);
}

//-----------------------------------------------------------------------------
SpinningJointHardwareInterface::Feedback::Feedback(
    const hardware_interface::ComponentInfo & joint_info):
  position(joint_info,hardware_interface::HW_IF_POSITION),
  velocity(joint_info,hardware_interface::HW_IF_VELOCITY),
  torque(joint_info,hardware_interface::HW_IF_EFFORT)
{

}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::Feedback::export_state_interfaces(std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  position.export_interface(state_interfaces);
  velocity.export_interface(state_interfaces);
  torque.export_interface(state_interfaces);
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::Feedback::set_state(const RotationalMotionState & state)
{
  position.set(state.position);
  velocity.set(state.velocity);
  torque.set(state.torque);
}

//-----------------------------------------------------------------------------
double SpinningJointHardwareInterface::get_command() const
{
  return command_.get();
}

//-----------------------------------------------------------------------------
void SpinningJointHardwareInterface::set_state(const RotationalMotionState & state)
{
   feedback_.set_state(state);
}

}
