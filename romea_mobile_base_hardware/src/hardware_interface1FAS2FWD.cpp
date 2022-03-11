#include "romea_mobile_base_hardware/hardware_interface1FAS2FWD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface1FAS2FWD::HardwareInterface1FAS2FWD(const hardware_interface::HardwareInfo & hardware_info,
                                                     const std::string & spinning_joint_command_interface_type):
  front_axle_steering_joint(hardware_info.joints[FRONT_AXLE_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint(hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  front_right_wheel_spinning_joint(hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  front_left_wheel_steering_joint_feedback(hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID],hardware_interface::HW_IF_POSITION),
  front_right_wheel_steering_joint_feedback(hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID],hardware_interface::HW_IF_POSITION),
  rear_left_wheel_spinning_joint_feedback(hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]),
  rear_right_wheel_spinning_joint_feedback(hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID])
{

}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface1FAS2FWD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_axle_steering_joint.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_left_wheel_steering_joint_feedback.export_interface(state_interfaces);
  front_right_wheel_steering_joint_feedback.export_interface(state_interfaces);
  rear_left_wheel_spinning_joint_feedback.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_feedback.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface1FAS2FWD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_axle_steering_joint.export_command_interface(command_interfaces);
  front_left_wheel_spinning_joint.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;
}

}

