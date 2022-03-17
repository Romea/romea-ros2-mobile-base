#include "romea_mobile_base_controllers/interfaces/controller_interface2WD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string left_wheel_steering_joint_param_name="left_wheel_steering_joint_name";
const std::string right_wheel_steering_joint_param_name="right_wheel_steering_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2WD::
ControllerInterface2WD(const MobileBaseInfo2WD & mobile_base_info,
                       const std::map<int, std::string> &joint_mappings,
                       LoanedCommandInterfaces & loaned_command_interfaces,
                       LoanedStateInterfaces & loaned_state_interfaces):
  left_spinning_joint_(loaned_command_interfaces,
                       loaned_state_interfaces,
                       joint_mappings.at(LEFT_WHEEL_SPINNING_JOINT_ID),
                       mobile_base_info.geometry.wheels.radius),
  right_spinning_joint_(loaned_command_interfaces,
                        loaned_state_interfaces,
                        joint_mappings.at(RIGHT_WHEEL_SPINNING_JOINT_ID),
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

//-----------------------------------------------------------------------------
void ControllerInterface2WD::declare_joints_mapping(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,right_wheel_steering_joint_param_name);
}

//-----------------------------------------------------------------------------
std::map<int,std::string> ControllerInterface2WD::get_joints_mapping(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  std::map<int,std::string> joint_mappings;
  joint_mappings[LEFT_WHEEL_SPINNING_JOINT_ID]=
      get_parameter<std::string>(node,parameters_ns,left_wheel_steering_joint_param_name);
  joint_mappings[RIGHT_WHEEL_SPINNING_JOINT_ID]=
      get_parameter<std::string>(node,parameters_ns,left_wheel_steering_joint_param_name);
  return joint_mappings;
}

}

