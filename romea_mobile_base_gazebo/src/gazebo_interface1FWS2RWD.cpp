#include "romea_mobile_base_gazebo/gazebo_interface1FWS2RWD.hpp"
#include <romea_mobile_base_hardware/hardware_interface1FWS2RWD.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface1FWS2RWD::GazeboInterface1FWS2RWD(gazebo::physics::ModelPtr parent_model,
                                                 const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & command_interface_type):
  front_wheel_steering_joint_(parent_model,hardware_info.joints[HardwareInterface1FWS2RWD::FRONT_WHEEL_STEERING_JOINT_ID]),
  front_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface1FWS2RWD::FRONT_WHEEL_STEERING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface1FWS2RWD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint_(parent_model,hardware_info.joints[HardwareInterface1FWS2RWD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{

}

//-----------------------------------------------------------------------------
SimulationState1FWS2RWD GazeboInterface1FWS2RWD::get_state() const
{
  return {front_wheel_steering_joint_.get_state(),
        front_wheel_spinning_joint_.get_state(),
        rear_left_wheel_spinning_joint_.get_state(),
        rear_right_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface1FWS2RWD::set_command(const SimulationCommand1FWS2RWD & command)
{
  front_wheel_steering_joint_.set_command(command.frontWheelSteeringAngle);
  front_wheel_spinning_joint_.set_command(command.frontWheelSetPoint);
  rear_left_wheel_spinning_joint_.set_command(command.rearLeftWheelSetPoint);
  rear_right_wheel_spinning_joint_.set_command(command.rearRightWheelSetPoint);
}



}
