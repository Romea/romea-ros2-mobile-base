#include "romea_mobile_base_controllers/interfaces/controller_interface1FWS2RWD.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface1FWS2RWD::
ControllerInterface1FWS2RWD(const MobileBaseInfo1FWS2RWD & mobile_base_info,
                            const std::map<std::string,std::string> & joint_mappings,
                            LoanedCommandInterfaces & loaned_command_interfaces,
                            LoanedStateInterfaces & loaned_state_interfaces):
  front_steering_joint_(loaned_command_interfaces,
                        loaned_state_interfaces,
                        joint_mappings.at("front_wheel_steering_joint_name")),
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
void ControllerInterface1FWS2RWD::setCommand(const OdometryFrame1FWS2RWD &command)
{
  front_steering_joint_.setCommand(command.frontWheelAngle);
  rear_left_spinning_joint_.setCommand(command.rearLeftWheelSpeed);
  rear_right_spinning_joint_.setCommand(command.rearRightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame1FWS2RWD ControllerInterface1FWS2RWD::getOdometryFrame() const
{
  OdometryFrame1FWS2RWD odometry;
  odometry.frontWheelAngle = front_steering_joint_.getMeasurement();
  odometry.rearLeftWheelSpeed = rear_left_spinning_joint_.getMeasurement();
  odometry.rearRightWheelSpeed = rear_right_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FWS2RWD::getCommandInterfaceNames()const
{
  return { front_steering_joint_.getCommandInterfaceName(),
        rear_left_spinning_joint_.getCommandInterfaceName(),
        rear_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FWS2RWD::getStateInterfaceNames()const
{
  return { front_steering_joint_.getStateInterfaceName(),
        rear_left_spinning_joint_.getStateInterfaceName(),
        rear_right_spinning_joint_.getStateInterfaceName()};
}

}
