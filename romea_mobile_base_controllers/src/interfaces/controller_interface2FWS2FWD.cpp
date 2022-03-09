#include "romea_mobile_base_controllers/interfaces/controller_interface2FWS2FWD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2FWS2FWD::
ControllerInterface2FWS2FWD(const MobileBaseInfo2FWS2FWD & mobile_base_info,
                            const std::map<std::string,std::string> & joint_mappings,
                            LoanedCommandInterfaces & loaned_command_interfaces,
                            LoanedStateInterfaces & loaned_state_interfaces):
  front_left_steering_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("front_left_wheel_steering_joint_name")),
  front_right_steering_joint_(loaned_command_interfaces,
                              loaned_state_interfaces,
                              joint_mappings.at("front_left_wheel_steering_joint_name")),
  front_left_spinning_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("front_left_wheel_spinning_joint_name"),
                             mobile_base_info.geometry.frontAxle.wheels.radius),
  front_right_spinning_joint_(loaned_command_interfaces,
                              loaned_state_interfaces,
                              joint_mappings.at("front_left_wheel_spinning_joint_name"),
                              mobile_base_info.geometry.frontAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2FWD::setCommand(const OdometryFrame2FWS2FWD &command)
{
  front_left_steering_joint_.setCommand(command.frontLeftWheelAngle);
  front_right_steering_joint_.setCommand(command.frontRightWheelAngle);
  front_left_spinning_joint_.setCommand(command.frontLeftWheelSpeed);
  front_right_spinning_joint_.setCommand(command.frontRightWheelSpeed);
}


//-----------------------------------------------------------------------------
OdometryFrame2FWS2FWD ControllerInterface2FWS2FWD::getOdometryFrame() const
{
  OdometryFrame2FWS2FWD odometry;
  odometry.frontLeftWheelAngle = front_left_steering_joint_.getMeasurement();
  odometry.frontRightWheelAngle = front_right_steering_joint_.getMeasurement();
  odometry.frontLeftWheelSpeed = front_left_spinning_joint_.getMeasurement();
  odometry.frontRightWheelSpeed = front_right_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2FWS2FWD::getCommandInterfaceNames()const
{
  return {
    front_left_steering_joint_.getCommandInterfaceName(),
        front_right_steering_joint_.getCommandInterfaceName(),
        front_left_spinning_joint_.getCommandInterfaceName(),
        front_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2FWS2FWD::getStateInterfaceNames()const
{
  return {
    front_left_steering_joint_.getStateInterfaceName(),
        front_right_steering_joint_.getStateInterfaceName(),
        front_left_spinning_joint_.getStateInterfaceName(),
        front_right_spinning_joint_.getStateInterfaceName()};
}

}

