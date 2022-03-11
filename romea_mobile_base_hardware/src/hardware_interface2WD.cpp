#include "romea_mobile_base_hardware/hardware_interface2WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2WD::HardwareInterface2WD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type):
  left_wheel_spinning_joint(hardware_info.joints[LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  right_wheel_spinning_joint(hardware_info.joints[RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_wheel_spinning_joint.export_command_interface(command_interfaces);
  right_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;
}

}

