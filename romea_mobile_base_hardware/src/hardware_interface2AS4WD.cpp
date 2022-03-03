#include "romea_mobile_base_hardware/hardware_interface2AS4WD.hpp"


namespace  {
const size_t FRONT_AXLE_STEERING_JOINT_ID=0;
const size_t REAR_AXLE_STEERING_JOINT_ID=1;
const size_t FRONT_LEFT_WHEEL_STEERING_JOINT_ID=2;
const size_t FRONT_RIGHT_WHEEL_STEERING_JOINT_ID=3;
const size_t REAR_LEFT_WHEEL_STEERING_JOINT_ID=4;
const size_t REAR_RIGHT_WHEEL_STEERING_JOINT_ID=5;
const size_t FRONT_LEFT_WHEEL_SPINNING_JOINT_ID=6;
const size_t FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID=7;
const size_t REAR_LEFT_WHEEL_SPINNING_JOINT_ID=8;
const size_t REAR_RIGHT_WHEEL_SPINNING_JOINT_ID=9;
}

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2AS4WD::HardwareInterface2AS4WD(const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & spinning_joint_command_interface_type):
  front_axle_steering_joint(hardware_info.joints[FRONT_AXLE_STEERING_JOINT_ID]),
  rear_axle_steering_joint(hardware_info.joints[REAR_AXLE_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint(hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  front_right_wheel_spinning_joint(hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  rear_left_wheel_spinning_joint(hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint(hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  front_left_wheel_steering_joint_feedback(hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID],hardware_interface::HW_IF_POSITION),
  front_right_wheel_steering_joint_feedback(hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID],hardware_interface::HW_IF_POSITION),
  rear_left_wheel_steering_joint_feedback(hardware_info.joints[REAR_LEFT_WHEEL_STEERING_JOINT_ID],hardware_interface::HW_IF_POSITION),
  rear_right_wheel_steering_joint_feedback(hardware_info.joints[REAR_RIGHT_WHEEL_STEERING_JOINT_ID],hardware_interface::HW_IF_POSITION)
{

}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2AS4WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_axle_steering_joint.export_state_interface(state_interfaces);
  rear_axle_steering_joint.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  front_left_wheel_steering_joint_feedback.export_interface(state_interfaces);
  front_right_wheel_steering_joint_feedback.export_interface(state_interfaces);
  rear_left_wheel_steering_joint_feedback.export_interface(state_interfaces);
  rear_right_wheel_steering_joint_feedback.export_interface(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2AS4WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_axle_steering_joint.export_command_interface(command_interfaces);
  rear_axle_steering_joint.export_command_interface(command_interfaces);
  front_left_wheel_spinning_joint.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;
}

}

