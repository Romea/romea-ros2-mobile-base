#include "romea_mobile_base_hardware/steering_joint_hardware_interface.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SteeringJointHardwareInterface::SteeringJointHardwareInterface(
    const hardware_interface::ComponentInfo & joint_info):
  command(joint_info,hardware_interface::HW_IF_POSITION),
  feedback(joint_info,hardware_interface::HW_IF_POSITION)
{

}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces)
{
  command.export_interface(command_interfaces);
}

//-----------------------------------------------------------------------------
void SteeringJointHardwareInterface::export_state_interface(
    std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  feedback.export_interface(state_interfaces);
}


}
