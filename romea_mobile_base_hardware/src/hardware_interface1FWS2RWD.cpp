#include "romea_mobile_base_hardware/hardware_interface1FWS2RWD.hpp"


//namespace  {
//const size_t FRONT_WHEEL_STEERING_JOINT_ID=0;
//const size_t FRONT_WHEEL_SPINNING_JOINT_ID=1;
//const size_t REAR_LEFT_WHEEL_SPINNING_JOINT_ID=2;
//const size_t REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=3;
//}

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface1FWS2RWD::HardwareInterface1FWS2RWD(const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & spinning_joint_command_interface_type):
  front_wheel_steering_joint(hardware_info.joints[FRONT_WHEEL_STEERING_JOINT_ID]),
  rear_left_wheel_spinning_joint(hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint(hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  front_wheel_spinning_joint_feedback(hardware_info.joints[FRONT_WHEEL_SPINNING_JOINT_ID])
{

}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface1FWS2RWD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_wheel_steering_joint.export_state_interface(state_interfaces);
  rear_left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_wheel_spinning_joint_feedback.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface1FWS2RWD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_wheel_steering_joint.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;
}

}

