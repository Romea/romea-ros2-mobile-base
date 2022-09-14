#include "romea_mobile_base_hardware/hardware_interface4WS4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface4WS4WD::HardwareInterface4WS4WD(const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & spinning_joint_command_interface_type):
  front_left_wheel_steering_joint_(hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint_(hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  rear_left_wheel_steering_joint_(hardware_info.joints[REAR_LEFT_WHEEL_STEERING_JOINT_ID]),
  rear_right_wheel_steering_joint_(hardware_info.joints[REAR_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint_(hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  front_right_wheel_spinning_joint_(hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  rear_left_wheel_spinning_joint_(hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type),
  rear_right_wheel_spinning_joint_(hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],spinning_joint_command_interface_type)
{

}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface4WS4WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  front_left_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_right_wheel_steering_joint_.export_state_interface(state_interfaces);
  rear_left_wheel_steering_joint_.export_state_interface(state_interfaces);
  rear_right_wheel_steering_joint_.export_state_interface(state_interfaces);
  front_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_left_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  rear_right_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface4WS4WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  front_left_wheel_steering_joint_.export_command_interface(command_interfaces);
  front_right_wheel_steering_joint_.export_command_interface(command_interfaces);
  rear_left_wheel_steering_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_steering_joint_.export_command_interface(command_interfaces);
  front_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  front_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_left_wheel_spinning_joint_.export_command_interface(command_interfaces);
  rear_right_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;
}

//-----------------------------------------------------------------------------
HardwareCommand4WS4WD HardwareInterface4WS4WD::get_command()const
{
  return { front_left_wheel_steering_joint_.get_command(),
        front_right_wheel_steering_joint_.get_command(),
        rear_left_wheel_steering_joint_.get_command(),
        rear_right_wheel_steering_joint_.get_command(),
        front_left_wheel_spinning_joint_.get_command(),
        front_right_wheel_spinning_joint_.get_command(),
        rear_left_wheel_spinning_joint_.get_command(),
        rear_right_wheel_spinning_joint_.get_command()};
}

//-----------------------------------------------------------------------------
void HardwareInterface4WS4WD::set_state(const HardwareState4WS4WD & hardware_state)
{
  front_left_wheel_steering_joint_.
      set_state(hardware_state.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.
      set_state(hardware_state.frontRightWheelSteeringAngle);
  rear_left_wheel_steering_joint_.
      set_state(hardware_state.rearLeftWheelSteeringAngle);
  rear_right_wheel_steering_joint_.
      set_state(hardware_state.rearRightWheelSteeringAngle);

  front_left_wheel_spinning_joint_.
      set_state(hardware_state.frontLeftWheelSpinningMotion);
  front_right_wheel_spinning_joint_.
      set_state(hardware_state.frontRightWheelSpinningMotion);
  rear_left_wheel_spinning_joint_.
      set_state(hardware_state.rearLeftWheelSpinningMotion);
  rear_right_wheel_spinning_joint_.
      set_state(hardware_state.rearRightWheelSpinningMotion);
}


}

