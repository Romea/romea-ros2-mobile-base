#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2TTD::HardwareInterface2TTD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type):
  left_sprocket_wheel_spinning_joint(hardware_info.joints[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  right_sprocket_wheel_spinning_joint(hardware_info.joints[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  left_idler_wheel_spinning_joint(hardware_info.joints[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  right_idler_wheel_spinning_joint(hardware_info.joints[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_left_roller_wheel_spinning_joint(hardware_info.joints[FRONT_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_roller_wheel_spinning_joint(hardware_info.joints[FRONT_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_roller_wheel_spinning_joint(hardware_info.joints[REAR_LEFT_ROLLER_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_roller_wheel_spinning_joint(hardware_info.joints[REAR_RIGHT_ROLLER_WHEEL_SPINNING_JOINT_ID],command_interface_type)

{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2TTD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  left_idler_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  right_idler_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_left_roller_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_right_roller_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  rear_left_roller_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  rear_right_roller_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  return state_interfaces;

}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2TTD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint.export_command_interface(command_interfaces);
  left_idler_wheel_spinning_joint.export_command_interface(command_interfaces);
  right_idler_wheel_spinning_joint.export_command_interface(command_interfaces);
  front_left_roller_wheel_spinning_joint.export_command_interface(command_interfaces);
  front_right_roller_wheel_spinning_joint.export_command_interface(command_interfaces);
  rear_left_roller_wheel_spinning_joint.export_command_interface(command_interfaces);
  rear_right_roller_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;

}

}

