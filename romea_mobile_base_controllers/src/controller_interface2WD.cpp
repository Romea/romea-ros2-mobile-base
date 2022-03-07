#include "romea_mobile_base_controllers/controller_interface2WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2WD::
ControllerInterface2WD(const MobileBaseInfo2WD & mobile_base_info,
                       const std::map<std::string,std::string> & joint_mappings,
                       LoanedCommandInterfaces & loaned_command_interfaces,
                       LoanedStateInterfaces & loaned_state_interfaces):
  left_spinning_joint_(loaned_command_interfaces,
                       loaned_state_interfaces,
                       joint_mappings.at("left_spinning_joint_name"),
                       mobile_base_info.geometry.wheels.radius),
  right_spinning_joint_(loaned_command_interfaces,
                        loaned_state_interfaces,
                        joint_mappings.at("right_spinning_joint_name"),
                        mobile_base_info.geometry.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::setCommand(const OdometryFrame2WD &command)
{
  left_spinning_joint_.setCommand(command.leftWheelSpeed);
  right_spinning_joint_.setCommand(command.rightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame2WD ControllerInterface2WD::getOdometryFrame() const
{
  OdometryFrame2WD odometry;
  odometry.leftWheelSpeed = left_spinning_joint_.getMeasurement();
  odometry.rightWheelSpeed = right_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2WD::getCommandInterfaceNames()const
{
  return {left_spinning_joint_.getCommandInterfaceName(),
        right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2WD::getStateInterfaceNames()const
{
  return {left_spinning_joint_.getStateInterfaceName(),
        right_spinning_joint_.getStateInterfaceName()};
}

}

