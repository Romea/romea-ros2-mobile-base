#include "romea_mobile_base_controllers/interfaces/controller_interface4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface4WD::
ControllerInterface4WD(const MobileBaseInfo4WD & mobile_base_info,
                       const std::map<std::string,std::string> & joint_mappings,
                       LoanedCommandInterfaces & loaned_command_interfaces,
                       LoanedStateInterfaces & loaned_state_interfaces):
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
void ControllerInterface4WD::setCommand(const OdometryFrame4WD &command)
{
  front_left_spinning_joint_.setCommand(command.frontLeftWheelSpeed);
  front_right_spinning_joint_.setCommand(command.frontRightWheelSpeed);
  rear_left_spinning_joint_.setCommand(command.rearLeftWheelSpeed);
  rear_right_spinning_joint_.setCommand(command.rearRightWheelSpeed);
}


//-----------------------------------------------------------------------------
OdometryFrame4WD ControllerInterface4WD::getOdometryFrame() const
{
  OdometryFrame4WD odometry;
  odometry.frontLeftWheelSpeed = front_left_spinning_joint_.getMeasurement();
  odometry.frontRightWheelSpeed = front_right_spinning_joint_.getMeasurement();
  odometry.rearLeftWheelSpeed = rear_left_spinning_joint_.getMeasurement();
  odometry.rearRightWheelSpeed = rear_right_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WD::getCommandInterfaceNames()const
{
  return {front_left_spinning_joint_.getCommandInterfaceName(),
        front_right_spinning_joint_.getCommandInterfaceName(),
        rear_left_spinning_joint_.getCommandInterfaceName(),
        rear_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WD::getStateInterfaceNames()const
{
  return {front_left_spinning_joint_.getStateInterfaceName(),
        front_right_spinning_joint_.getStateInterfaceName(),
        rear_left_spinning_joint_.getStateInterfaceName(),
        rear_right_spinning_joint_.getStateInterfaceName()};
}

}
