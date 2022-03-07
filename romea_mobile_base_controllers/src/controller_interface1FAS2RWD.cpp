#include "romea_mobile_base_controllers/controller_interface1FAS2RWD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface1FAS2RWD::
ControllerInterface1FAS2RWD(const MobileBaseInfo1FAS2RWD & mobile_base_info,
                            const std::map<std::string,std::string> & joint_mappings,
                            LoanedCommandInterfaces & loaned_command_interfaces,
                            LoanedStateInterfaces & loaned_state_interfaces):
  front_steering_joint_(loaned_command_interfaces,
                        loaned_state_interfaces,
                        joint_mappings.at("front_axle_steering_joint_name")),
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
void ControllerInterface1FAS2RWD::setCommand(const OdometryFrame1FAS2RWD &command)
{
  front_steering_joint_.setCommand(command.frontAxleSteeringAngle);
  rear_left_spinning_joint_.setCommand(command.rearLeftWheelSpeed);
  rear_right_spinning_joint_.setCommand(command.rearRightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame1FAS2RWD ControllerInterface1FAS2RWD::getOdometryFrame() const
{
  OdometryFrame1FAS2RWD odometry;
  odometry.frontAxleSteeringAngle = front_steering_joint_.getMeasurement();
  odometry.rearLeftWheelSpeed = rear_left_spinning_joint_.getMeasurement();
  odometry.rearRightWheelSpeed = rear_right_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2RWD::getCommandInterfaceNames()const
{
  return {front_steering_joint_.getCommandInterfaceName(),
        rear_left_spinning_joint_.getCommandInterfaceName(),
        rear_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2RWD::getStateInterfaceNames()const
{
  return {front_steering_joint_.getStateInterfaceName(),
        rear_left_spinning_joint_.getStateInterfaceName(),
        rear_right_spinning_joint_.getStateInterfaceName()};
}


}

