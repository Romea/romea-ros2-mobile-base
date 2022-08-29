#include "romea_mobile_base_gazebo/gazebo_interface1FASxxx.hpp"
#include <romea_mobile_base_simulation/simulation_interface1FAS2FWD.hpp>


namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface1FASxxx::GazeboInterface1FASxxx(gazebo::physics::ModelPtr parent_model,
                                               const hardware_interface::HardwareInfo & hardware_info,
                                               const std::string & command_interface_type):
  front_axle_steering_joint_(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_AXLE_STEERING_JOINT_ID]),
  front_left_wheel_steering_joint_(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint_(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{

}

//-----------------------------------------------------------------------------
SimulationState1FASxxx GazeboInterface1FASxxx::get_state() const
{
  return {front_axle_steering_joint_.get_state(),
        front_left_wheel_steering_joint_.get_state(),
        front_right_wheel_steering_joint_.get_state(),
        front_left_wheel_spinning_joint_.get_state(),
        front_right_wheel_spinning_joint_.get_state(),
        rear_left_wheel_spinning_joint_.get_state(),
        rear_right_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface1FASxxx::set_command(const SimulationCommand1FASxxx & command)
{
  front_axle_steering_joint_.set_command(command.frontAxleSteeringAngle);
  front_left_wheel_steering_joint_.set_command(command.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_command(command.frontRightWheelSteeringAngle);
  front_left_wheel_spinning_joint_.set_command(command.frontLeftWheelSetPoint);
  front_right_wheel_spinning_joint_.set_command(command.frontRightWheelSetPoint);
  rear_left_wheel_spinning_joint_.set_command(command.rearLeftWheelSetPoint);
  rear_right_wheel_spinning_joint_.set_command(command.rearRightWheelSetPoint);
}

}
