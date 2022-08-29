#include "romea_mobile_base_gazebo/gazebo_interface4WS4WD.hpp"
#include <romea_mobile_base_hardware/hardware_interface4WS4WD.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface4WS4WD::GazeboInterface4WS4WD(gazebo::physics::ModelPtr parent_model,
                                             const hardware_interface::HardwareInfo & hardware_info,
                                             const std::string & command_interface_type):
  front_left_wheel_steering_joint_(parent_model,hardware_info.joints[HardwareInterface4WS4WD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint_(parent_model,hardware_info.joints[HardwareInterface4WS4WD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  rear_left_wheel_steering_joint_(parent_model,hardware_info.joints[HardwareInterface4WS4WD::REAR_LEFT_WHEEL_STEERING_JOINT_ID]),
  rear_right_wheel_steering_joint_(parent_model,hardware_info.joints[HardwareInterface4WS4WD::REAR_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface4WS4WD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface4WS4WD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface4WS4WD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface4WS4WD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{

}

//-----------------------------------------------------------------------------
SimulationState4WS4WD GazeboInterface4WS4WD::get_state() const
{

  return {front_left_wheel_steering_joint_.get_state(),
        front_right_wheel_steering_joint_.get_state(),
        rear_left_wheel_steering_joint_.get_state(),
        rear_right_wheel_steering_joint_.get_state(),
        front_left_wheel_spinning_joint_.get_state(),
        front_right_wheel_spinning_joint_.get_state(),
        rear_left_wheel_spinning_joint_.get_state(),
        rear_right_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface4WS4WD::set_command(const SimulationCommand4WS4WD &command)
{
  front_left_wheel_steering_joint_.set_command(command.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_command(command.frontRightWheelSteeringAngle);
  rear_left_wheel_steering_joint_.set_command(command.rearLeftWheelSteeringAngle);
  rear_right_wheel_steering_joint_.set_command(command.rearRightWheelSteeringAngle);
  front_left_wheel_spinning_joint_.set_command(command.frontLeftWheelSetPoint);
  front_right_wheel_spinning_joint_.set_command(command.frontRightWheelSetPoint);
  rear_left_wheel_spinning_joint_.set_command(command.rearLeftWheelSetPoint);
  rear_right_wheel_spinning_joint_.set_command(command.rearRightWheelSetPoint);
}




}
