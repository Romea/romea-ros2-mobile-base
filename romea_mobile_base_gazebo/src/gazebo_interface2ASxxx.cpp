#include "romea_mobile_base_gazebo/gazebo_interface2ASxxx.hpp"
#include <romea_mobile_base_hardware/hardware_interface2AS4WD.hpp>


namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2ASxxx::GazeboInterface2ASxxx(gazebo::physics::ModelPtr parent_model,
                                             const hardware_interface::HardwareInfo & hardware_info,
                                             const std::string & command_interface_type):
  front_axle_steering_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_AXLE_STEERING_JOINT_ID)),
  rear_axle_steering_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_AXLE_STEERING_JOINT_ID)),
  front_left_wheel_steering_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID)),
  front_right_wheel_steering_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID)),
  rear_left_wheel_steering_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_LEFT_WHEEL_STEERING_JOINT_ID)),
  rear_right_wheel_steering_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_RIGHT_WHEEL_STEERING_JOINT_ID)),
  front_left_wheel_spinning_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID),command_interface_type),
  front_right_wheel_spinning_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID),command_interface_type),
  rear_left_wheel_spinning_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID),command_interface_type),
  rear_right_wheel_spinning_joint_(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID),command_interface_type)
{

}

//-----------------------------------------------------------------------------
SimulationState2ASxxx GazeboInterface2ASxxx::get_state() const
{
  return{front_axle_steering_joint_.get_state(),
        front_left_wheel_steering_joint_.get_state(),
        front_right_wheel_steering_joint_.get_state(),
        rear_axle_steering_joint_.get_state(),
        rear_left_wheel_steering_joint_.get_state(),
        rear_right_wheel_steering_joint_.get_state(),
        front_left_wheel_spinning_joint_.get_state(),
        front_right_wheel_spinning_joint_.get_state(),
        rear_left_wheel_spinning_joint_.get_state(),
        rear_right_wheel_spinning_joint_.get_state()};
}

//-----------------------------------------------------------------------------
void GazeboInterface2ASxxx::set_command(const SimulationCommand2ASxxx & command)
{
  front_axle_steering_joint_.set_command(command.frontAxleSteeringAngle);
  rear_axle_steering_joint_.set_command(command.rearAxleSteeringAngle);

  front_left_wheel_steering_joint_.set_command(command.frontLeftWheelSteeringAngle);
  front_right_wheel_steering_joint_.set_command(command.frontRightWheelSteeringAngle);
  rear_left_wheel_steering_joint_.set_command(command.rearLeftWheelSteeringAngle);
  rear_right_wheel_steering_joint_.set_command(command.rearRightWheelSteeringAngle);
  front_left_wheel_spinning_joint_.set_command(command.frontLeftWheelSpinningSetPoint);
  front_right_wheel_spinning_joint_.set_command(command.frontRightWheelSpinningSetPoint);
  rear_left_wheel_spinning_joint_.set_command(command.rearLeftWheelSpinningSetPoint);
  rear_right_wheel_spinning_joint_.set_command(command.rearRightWheelSpinningSetPoint);
}

}


