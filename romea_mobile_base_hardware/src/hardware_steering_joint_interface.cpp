#include "romea_mobile_base_hardware/hardware_steering_joint_interface.hpp"
#include "romea_mobile_base_hardware/hardware_joint_info.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareSteeringJointInterface::HardwareSteeringJointInterface(
    const hardware_interface::ComponentInfo & joint_info):
  command(joint_info,hardware_interface::HW_IF_POSITION),
  feedback(joint_info,hardware_interface::HW_IF_POSITION)
{

}

//-----------------------------------------------------------------------------
void HardwareSteeringJointInterface::export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces)
{
  command.export_interface(command_interfaces);
}



}
