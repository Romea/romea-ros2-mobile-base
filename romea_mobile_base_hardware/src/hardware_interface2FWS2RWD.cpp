#include "romea_mobile_base_hardware/hardware_interface2FWS2RWD.hpp"

namespace  {
const size_t FRONT_LEFT_WHEEL_STEERING_JOINT_ID=0;
const size_t FRONT_RIGHT_WHEEL_STEERING_JOINT_ID=1;
const size_t FRONT_LEFT_WHEEL_SPINNING_JOINT_ID=2;
const size_t FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID=3;
const size_t REAR_LEFT_WHEEL_SPINNING_JOINT_ID=4;
const size_t REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=5;
}

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2FWS2RWD::HardwareInterface2FWS2RWD(const hardware_interface::HardwareInfo & hardware_info,
                                                     const std::string & spinning_joint_command_interface_type):
  front_left_wheel_steering_joint(hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint(hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  rear_left_wheel_spinning_joint(hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint(hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  front_left_wheel_spinning_joint_feedback(hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]),
  front_right_wheel_spinning_joint_feedback(hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID])
{

}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> HardwareInterface2FWS2RWD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_left_wheel_steering_joint.export_state_interface(state_interfaces);
  front_right_wheel_steering_joint.export_state_interface(state_interfaces);
  rear_left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_left_wheel_spinning_joint_feedback.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_feedback.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface> HardwareInterface2FWS2RWD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_left_wheel_steering_joint.export_command_interface(command_interfaces);
  front_right_wheel_steering_joint.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;
}


}

