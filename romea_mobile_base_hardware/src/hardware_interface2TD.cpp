#include "romea_mobile_base_hardware/hardware_interface2TD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2TD::HardwareInterface2TD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type):
  left_sprocket_wheel_spinning_joint(hardware_info.joints[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  right_sprocket_wheel_spinning_joint(hardware_info.joints[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  left_idler_wheel_spinning_joint_feedback(hardware_info.joints[LEFT_IDLER_WHEEL_SPINNING_JOINT_ID]),
  right_idler_wheel_spinning_joint_feedback(hardware_info.joints[RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID])
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2TD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  left_idler_wheel_spinning_joint_feedback.export_state_interfaces(state_interfaces);
  right_idler_wheel_spinning_joint_feedback.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2TD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;
}

}

