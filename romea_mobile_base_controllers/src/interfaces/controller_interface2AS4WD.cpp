#include "romea_mobile_base_controllers/interfaces/controller_interface2AS4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2AS4WD::
ControllerInterface2AS4WD(const MobileBaseInfo2AS4WD & mobile_base_info,
                          const std::map<std::string,std::string> & joint_mappings,
                          LoanedCommandInterfaces & loaned_command_interfaces,
                          LoanedStateInterfaces & loaned_state_interfaces):
  front_steering_joint_(loaned_command_interfaces,
                        loaned_state_interfaces,
                        joint_mappings.at("front_axle_steering_joint_name")),
  rear_steering_joint_(loaned_command_interfaces,
                       loaned_state_interfaces,
                       joint_mappings.at("rear_axle_steering_joint_name")),
  front_left_spinning_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("front_left_wheel_spinning_joint_name"),
                             mobile_base_info.geometry.frontAxle.wheels.radius),
  front_right_spinning_joint_(loaned_command_interfaces,
                              loaned_state_interfaces,
                              joint_mappings.at("front_left_wheel_spinning_joint_name"),
                              mobile_base_info.geometry.frontAxle.wheels.radius),
  rear_left_spinning_joint_(loaned_command_interfaces,
                            loaned_state_interfaces,
                            joint_mappings.at("rear_left_wheel_spinning_joint_name"),
                            mobile_base_info.geometry.rearAxle.wheels.radius),
  rear_right_spinning_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("rear_left_wheel_spinning_joint_name"),
                             mobile_base_info.geometry.rearAxle.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface2AS4WD::setCommand(const OdometryFrame2AS4WD &command)
{
  front_steering_joint_.setCommand(command.frontAxleSteeringAngle);
  rear_steering_joint_.setCommand(command.rearAxleSteeringAngle);
  front_left_spinning_joint_.setCommand(command.frontLeftWheelSpeed);
  front_right_spinning_joint_.setCommand(command.frontRightWheelSpeed);
  rear_left_spinning_joint_.setCommand(command.rearLeftWheelSpeed);
  rear_right_spinning_joint_.setCommand(command.rearRightWheelSpeed);
}


//-----------------------------------------------------------------------------
OdometryFrame2AS4WD ControllerInterface2AS4WD::getOdometryFrame() const
{
  OdometryFrame2AS4WD odometry;
  odometry.frontAxleSteeringAngle = front_steering_joint_.getMeasurement();
  odometry.frontAxleSteeringAngle = rear_steering_joint_.getMeasurement();
  odometry.frontLeftWheelSpeed = front_left_spinning_joint_.getMeasurement();
  odometry.frontRightWheelSpeed = front_right_spinning_joint_.getMeasurement();
  odometry.rearLeftWheelSpeed = rear_left_spinning_joint_.getMeasurement();
  odometry.rearRightWheelSpeed = rear_right_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2AS4WD::getCommandInterfaceNames()const
{
  return { front_steering_joint_.getCommandInterfaceName(),
        rear_steering_joint_.getCommandInterfaceName(),
        front_left_spinning_joint_.getCommandInterfaceName(),
        front_right_spinning_joint_.getCommandInterfaceName(),
        rear_left_spinning_joint_.getCommandInterfaceName(),
        rear_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2AS4WD::getStateInterfaceNames()const
{
  return { front_steering_joint_.getStateInterfaceName(),
        rear_steering_joint_.getStateInterfaceName(),
        front_left_spinning_joint_.getStateInterfaceName(),
        front_right_spinning_joint_.getStateInterfaceName(),
        rear_left_spinning_joint_.getStateInterfaceName(),
        rear_right_spinning_joint_.getStateInterfaceName()};

}


}

