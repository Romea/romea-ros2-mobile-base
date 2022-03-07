#include "romea_mobile_base_controllers/controller_interface4WS4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface4WS4WD::
ControllerInterface4WS4WD(const MobileBaseInfo4WS4WD & mobile_base_info,
                          const std::map<std::string,std::string> & joint_mappings,
                          LoanedCommandInterfaces & loaned_command_interfaces,
                          LoanedStateInterfaces & loaned_state_interfaces):
  front_left_steering_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("front_left_wheel_steering_joint_name")),
  front_right_steering_joint_(loaned_command_interfaces,
                              loaned_state_interfaces,
                              joint_mappings.at("front_right_wheel_steering_joint_name")),
  rear_left_steering_joint_(loaned_command_interfaces,
                            loaned_state_interfaces,
                            joint_mappings.at("rear_left_wheel_steering_joint_name")),
  rear_right_steering_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("rear_right_wheel_steering_joint_name")),
  front_left_spinning_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("front_left_wheel_spinning_joint_name"),
                             mobile_base_info.geometry.frontAxle.wheels.radius),
  front_right_spinning_joint_(loaned_command_interfaces,
                              loaned_state_interfaces,
                              joint_mappings.at("front_right_wheel_spinning_joint_name"),
                              mobile_base_info.geometry.frontAxle.wheels.radius),
  rear_left_spinning_joint_(loaned_command_interfaces,
                            loaned_state_interfaces,
                            joint_mappings.at("rear_left_wheel_spinning_joint_name"),
                            mobile_base_info.geometry.rearAxle.wheels.radius),
  rear_right_spinning_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("rear_right_wheel_spinning_joint_name"),
                             mobile_base_info.geometry.rearAxle.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface4WS4WD::setCommand(const OdometryFrame4WS4WD &command)
{
  front_left_steering_joint_.setCommand(command.frontLeftWheelAngle);
  front_right_steering_joint_.setCommand(command.frontRightWheelAngle);
  rear_left_steering_joint_.setCommand(command.rearLeftWheelAngle);
  rear_right_steering_joint_.setCommand(command.rearRightWheelAngle);
  front_left_spinning_joint_.setCommand(command.frontLeftWheelSpeed);
  front_right_spinning_joint_.setCommand(command.frontRightWheelSpeed);
  rear_left_spinning_joint_.setCommand(command.rearLeftWheelSpeed);
  rear_right_spinning_joint_.setCommand(command.rearRightWheelSpeed);
}


//-----------------------------------------------------------------------------
OdometryFrame4WS4WD ControllerInterface4WS4WD::getOdometryFrame() const
{
  OdometryFrame4WS4WD odometry;
  odometry.frontLeftWheelAngle = front_left_steering_joint_.getMeasurement();
  odometry.frontRightWheelAngle = front_right_steering_joint_.getMeasurement();
  odometry.rearLeftWheelAngle = rear_left_steering_joint_.getMeasurement();
  odometry.rearRightWheelAngle = rear_right_steering_joint_.getMeasurement();
  odometry.frontLeftWheelSpeed = front_left_spinning_joint_.getMeasurement();
  odometry.frontRightWheelSpeed = front_right_spinning_joint_.getMeasurement();
  odometry.rearLeftWheelSpeed = rear_left_spinning_joint_.getMeasurement();
  odometry.rearRightWheelSpeed = rear_right_spinning_joint_.getMeasurement();
  return odometry;
}

std::vector<std::string> ControllerInterface4WS4WD::getCommandInterfaceNames()const
{
  return {front_left_steering_joint_.getCommandInterfaceName(),
        front_right_steering_joint_.getCommandInterfaceName(),
        rear_left_steering_joint_.getCommandInterfaceName(),
        rear_right_steering_joint_.getCommandInterfaceName(),
        front_left_spinning_joint_.getCommandInterfaceName(),
        front_right_spinning_joint_.getCommandInterfaceName(),
        rear_left_spinning_joint_.getCommandInterfaceName(),
        rear_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WS4WD::getStateInterfaceNames()const
{
  return {front_left_steering_joint_.getStateInterfaceName(),
        front_right_steering_joint_.getStateInterfaceName(),
        rear_left_steering_joint_.getStateInterfaceName(),
        rear_right_steering_joint_.getStateInterfaceName(),
        front_left_spinning_joint_.getStateInterfaceName(),
        front_right_spinning_joint_.getStateInterfaceName(),
        rear_left_spinning_joint_.getStateInterfaceName(),
        rear_right_spinning_joint_.getStateInterfaceName()};
}

}

