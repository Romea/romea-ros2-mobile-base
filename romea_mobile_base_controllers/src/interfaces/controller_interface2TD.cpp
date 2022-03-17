#include "romea_mobile_base_controllers/interfaces/controller_interface2TD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string left_sprocket_wheel_steering_joint_param_name="left_sprocket_wheel_steering_joint_name";
const std::string right_sprocket_wheel_steering_joint_param_name="right_sprocket_wheel_steering_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2TD::
ControllerInterface2TD(const MobileBaseInfo2TD &mobile_base_info,
                       const std::map<int, std::string> &joint_mappings,
                       LoanedCommandInterfaces & loaned_command_interfaces,
                       LoanedStateInterfaces & loaned_state_interfaces):
  left_sprocket_spinning_joint_(loaned_command_interfaces,
                                loaned_state_interfaces,
                                joint_mappings.at(LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID),
                                mobile_base_info.geometry.tracks.sprocketWheel.radius+
                                mobile_base_info.geometry.tracks.thickness),
  right_sprocket_spinning_joint_(loaned_command_interfaces,
                                 loaned_state_interfaces,
                                 joint_mappings.at(RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID),
                                 mobile_base_info.geometry.tracks.sprocketWheel.radius+
                                 mobile_base_info.geometry.tracks.thickness)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::setCommand(const OdometryFrame2WD &command)
{
  left_sprocket_spinning_joint_.setCommand(command.leftWheelSpeed);
  right_sprocket_spinning_joint_.setCommand(command.rightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame2WD ControllerInterface2TD::getOdometryFrame() const
{
  OdometryFrame2WD odometry;
  odometry.leftWheelSpeed = left_sprocket_spinning_joint_.getMeasurement();
  odometry.rightWheelSpeed = right_sprocket_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2TD::getCommandInterfaceNames()const
{
  return {left_sprocket_spinning_joint_.getCommandInterfaceName(),
        right_sprocket_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2TD::getStateInterfaceNames()const
{
  return {left_sprocket_spinning_joint_.getStateInterfaceName(),
        right_sprocket_spinning_joint_.getStateInterfaceName()};
}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::declare_joints_mapping(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,left_sprocket_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,right_sprocket_wheel_steering_joint_param_name);
}

//-----------------------------------------------------------------------------
std::map<int,std::string> ControllerInterface2TD::get_joints_mapping(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  std::map<int,std::string> joint_mappings;
  joint_mappings[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID]=
      get_parameter<std::string>(node,parameters_ns,left_sprocket_wheel_steering_joint_param_name);
  joint_mappings[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID]=
      get_parameter<std::string>(node,parameters_ns,left_sprocket_wheel_steering_joint_param_name);
  return joint_mappings;
}

}

