#include "romea_mobile_base_controllers/controller_interface1FAS2FWD.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface1FAS2FWD::
ControllerInterface1FAS2FWD(const MobileBaseInfo1FAS2FWD & mobile_base_info,
                            const std::map<std::string,std::string> & joint_mappings,
                            LoanedCommandInterfaces & loaned_command_interfaces,
                            LoanedStateInterfaces & loaned_state_interfaces):
  front_steering_joint_(loaned_command_interfaces,
                        loaned_state_interfaces,
                        joint_mappings.at("front_axle_steering_joint_name")),
  front_left_spinning_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at("front_left_wheel_spinning_joint_name"),
                             mobile_base_info.geometry.rearAxle.wheels.radius),
  front_right_spinning_joint_(loaned_command_interfaces,
                              loaned_state_interfaces,
                              joint_mappings.at("front_left_wheel_spinning_joint_name"),
                              mobile_base_info.geometry.rearAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2FWD::setCommand(const OdometryFrame1FAS2FWD &command)
{
  front_steering_joint_.setCommand(command.frontAxleSteeringAngle);
  front_left_spinning_joint_.setCommand(command.frontLeftWheelSpeed);
  front_right_spinning_joint_.setCommand(command.frontRightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame1FAS2FWD ControllerInterface1FAS2FWD::getOdometryFrame() const
{
  OdometryFrame1FAS2FWD odometry;
  odometry.frontAxleSteeringAngle = front_steering_joint_.getMeasurement();
  odometry.frontLeftWheelSpeed = front_left_spinning_joint_.getMeasurement();
  odometry.frontRightWheelSpeed = front_right_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2FWD::getCommandInterfaceNames()const
{
  return  {  front_steering_joint_.getCommandInterfaceName(),
        front_left_spinning_joint_.getCommandInterfaceName(),
        front_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2FWD::getStateInterfaceNames()const
{
  return  {  front_steering_joint_.getStateInterfaceName(),
        front_left_spinning_joint_.getStateInterfaceName(),
        front_right_spinning_joint_.getCommandInterfaceName()};
}

}

