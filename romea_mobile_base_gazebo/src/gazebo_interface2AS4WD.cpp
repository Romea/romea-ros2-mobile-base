#include "romea_mobile_base_hardware_interfaces/hardware_interface2AS4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2AS4WD::HardwareInterface2AS4WD(const Parameters & parameters,
                                                 const std::string & joints_prefix,
                                                 const hardware_interface::HardwareInfo & hardware_info):
  front_steering_joint_(nullptr),
  rear_steering_joint_(nullptr),
  front_left_spinning_joint_(nullptr),
  front_right_spinning_joint_(nullptr),
  rear_left_spinning_joint_(nullptr),
  rear_right_spinning_joint_(nullptr)
{

  front_steering_joint_ = makeSteeringJointHarwareInterface(
        hardware_info, joints_prefix + parameters.front_axle_steering_joint_name);

  rear_steering_joint_ = makeSteeringJointHarwareInterface(
        hardware_info, joints_prefix + parameters.rear_axle_steering_joint_name);

  front_left_spinning_joint_ = makeSpinningJointHarwareInterface(
        hardware_info, joints_prefix + parameters.front_left_wheel_spinning_joint_name);

  front_right_spinning_joint_ = makeSpinningJointHarwareInterface(
        hardware_info, joints_prefix + parameters.front_left_wheel_spinning_joint_name);

  rear_left_spinning_joint_ = makeSpinningJointHarwareInterface(
        hardware_info, joints_prefix + parameters.rear_left_wheel_spinning_joint_name);

  rear_right_spinning_joint_ = makeSpinningJointHarwareInterface(
        hardware_info, joints_prefix + parameters.rear_left_wheel_spinning_joint_name);

}

//-----------------------------------------------------------------------------
void HardwareInterface2AS4WD::setMeasurement(const OdometryFrame2AS4WD &measurement)
{
  front_steering_joint_->setMeasurement(measurement.frontAxleSteeringAngle);
  rear_steering_joint_->setMeasurement(measurement.rearAxleSteeringAngle);
  front_left_spinning_joint_->setMeasurement(measurement.frontLeftWheelSpeed);
  front_right_spinning_joint_->setMeasurement(measurement.frontRightWheelSpeed);
  rear_left_spinning_joint_->setMeasurement(measurement.rearLeftWheelSpeed);
  rear_right_spinning_joint_->setMeasurement(measurement.rearRightWheelSpeed);
}


//-----------------------------------------------------------------------------
OdometryFrame2AS4WD HardwareInterface2AS4WD::getCommand() const
{
  OdometryFrame2AS4WD command;
  command.frontAxleSteeringAngle = front_steering_joint_->getCommand();
  command.frontAxleSteeringAngle = rear_steering_joint_->getCommand();
  command.frontLeftWheelSpeed = front_left_spinning_joint_->getCommand();
  command.frontRightWheelSpeed = front_right_spinning_joint_->getCommand();
  command.rearLeftWheelSpeed = rear_left_spinning_joint_->getCommand();
  command.rearRightWheelSpeed = rear_right_spinning_joint_->getCommand();
  return command;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface> HardwareInterface2AS4WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(front_steering_joint_->exportStateInterface());
  state_interfaces.emplace_back(rear_steering_joint_->exportStateInterface());
  state_interfaces.emplace_back(front_left_spinning_joint_->exportStateInterface());
  state_interfaces.emplace_back(front_right_spinning_joint_->exportStateInterface());
  state_interfaces.emplace_back(rear_left_spinning_joint_->exportStateInterface());
  state_interfaces.emplace_back(rear_right_spinning_joint_->exportStateInterface());
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface> HardwareInterface2AS4WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(front_steering_joint_->exportCommandInterface());
  command_interfaces.emplace_back(rear_steering_joint_->exportCommandInterface());
  command_interfaces.emplace_back(front_left_spinning_joint_->exportCommandInterface());
  command_interfaces.emplace_back(front_right_spinning_joint_->exportCommandInterface());
  command_interfaces.emplace_back(rear_left_spinning_joint_->exportCommandInterface());
  command_interfaces.emplace_back(rear_right_spinning_joint_->exportCommandInterface());
  return command_interfaces;
}


}

