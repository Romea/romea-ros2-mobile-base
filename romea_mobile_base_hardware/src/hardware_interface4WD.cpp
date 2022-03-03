#include "romea_mobile_base_hardware/hardware_interface4WD.hpp"

namespace  {
const size_t FRONT_LEFT_WHEEL_SPINNING_JOINT_ID=0;
const size_t FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID=1;
const size_t REAR_LEFT_WHEEL_SPINNING_JOINT_ID=2;
const size_t REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=3;
}

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface4WD::HardwareInterface4WD(const hardware_interface::HardwareInfo & hardware_info,
                                           const std::string & command_interface_type):
  front_left_wheel_spinning_joint(hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_wheel_spinning_joint(hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint(hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint(hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> HardwareInterface4WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface> HardwareInterface4WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_left_wheel_spinning_joint.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;
}


}
