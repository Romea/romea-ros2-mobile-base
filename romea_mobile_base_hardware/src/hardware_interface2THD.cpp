#include "romea_mobile_base_hardware/hardware_interface2THD.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2THD::HardwareInterface2THD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type):
  left_sprocket_wheel_spinning_joint_(hardware_info.joints[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  right_sprocket_wheel_spinning_joint_(hardware_info.joints[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_left_idler_wheel_spinning_joint_feedback_(hardware_info.joints[FRONT_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID]),
  front_right_idler_wheel_spinning_joint_feedback_(hardware_info.joints[FRONT_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID]),
  rear_left_idler_wheel_spinning_joint_feedback_(hardware_info.joints[REAR_LEFT_IDLER_WHEEL_SPINNING_JOINT_ID]),
  rear_right_idler_wheel_spinning_joint_feedback_(hardware_info.joints[REAR_RIGHT_IDLER_WHEEL_SPINNING_JOINT_ID])
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2THD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  right_sprocket_wheel_spinning_joint_.export_state_interfaces(state_interfaces);
  front_left_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  front_right_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_left_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  rear_right_idler_wheel_spinning_joint_feedback_.export_state_interfaces(state_interfaces);
  return state_interfaces;

}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2THD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  right_sprocket_wheel_spinning_joint_.export_command_interface(command_interfaces);
  return command_interfaces;

}

//-----------------------------------------------------------------------------
HardwareCommand2TD HardwareInterface2THD::get_command()const
{
  return {left_sprocket_wheel_spinning_joint_.get_command(),
        right_sprocket_wheel_spinning_joint_.get_command()};

}

//-----------------------------------------------------------------------------
void HardwareInterface2THD::set_state(const HardwareState2TD & hardware_state)
{
  left_sprocket_wheel_spinning_joint_.
      set_state(hardware_state.leftSprocketWheelSpinMotion);
  right_sprocket_wheel_spinning_joint_.
      set_state(hardware_state.rightSprocketWheelSpinMotion);
}

//-----------------------------------------------------------------------------
void HardwareInterface2THD::set_state(const HardwareState2TD & hardware_state,
                                      const RotationalMotionState & front_left_idler_wheel_set_point,
                                      const RotationalMotionState & front_right_idler_wheel_set_point,
                                      const RotationalMotionState & rear_left_idler_wheel_set_point,
                                      const RotationalMotionState & rear_right_idler_wheel_set_point)
{
  set_state(hardware_state);

  front_left_idler_wheel_spinning_joint_feedback_.
      set_state(front_left_idler_wheel_set_point);
  front_right_idler_wheel_spinning_joint_feedback_.
      set_state(front_right_idler_wheel_set_point);
  rear_left_idler_wheel_spinning_joint_feedback_.
      set_state(rear_left_idler_wheel_set_point);
  rear_right_idler_wheel_spinning_joint_feedback_.
      set_state(rear_right_idler_wheel_set_point);
}

}

