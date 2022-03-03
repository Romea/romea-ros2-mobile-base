#include "romea_mobile_base_hardware/hardware_spinning_joint_interface.hpp"
#include "romea_mobile_base_hardware/hardware_joint_info.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareSpinningJointInterface::HardwareSpinningJointInterface(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & command_interface_type):
  command(joint_info,command_interface_type),
  feedback(joint_info)
{

}

//-----------------------------------------------------------------------------
void HardwareSpinningJointInterface::export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces)
{
  command.export_interface(command_interfaces);
}

//-----------------------------------------------------------------------------
void HardwareSpinningJointInterface::export_state_interfaces(
    std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  feedback.export_state_interfaces(state_interfaces);
}

//-----------------------------------------------------------------------------
HardwareSpinningJointInterface::Feedback::Feedback(
    const hardware_interface::ComponentInfo & joint_info):
  position(joint_info,hardware_interface::HW_IF_POSITION),
  velocity(joint_info,hardware_interface::HW_IF_VELOCITY),
  torque(joint_info,hardware_interface::HW_IF_EFFORT)
{

}

//-----------------------------------------------------------------------------
void HardwareSpinningJointInterface::Feedback::export_state_interfaces(std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  position.export_interface(state_interfaces);
  velocity.export_interface(state_interfaces);
  torque.export_interface(state_interfaces);
}


}
