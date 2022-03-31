#include "romea_mobile_base_controllers/interfaces/controller_interface2TD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string left_sprocket_wheel_spinning_joint_param_name="left_sprocket_wheel_spinning_joint_name";
const std::string right_sprocket_wheel_spinning_joint_param_name="right_sprocket_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2TD::ControllerInterface2TD(const MobileBaseInfo2TD &mobile_base_info,
                                               const std::vector<std::string> &joints_names):
  left_sprocket_spinning_joint_(joints_names[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
                                mobile_base_info.geometry.tracks.sprocketWheel.radius+
                                mobile_base_info.geometry.tracks.thickness),
  right_sprocket_spinning_joint_(joints_names[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],
                                 mobile_base_info.geometry.tracks.sprocketWheel.radius+
                                 mobile_base_info.geometry.tracks.thickness)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::register_loaned_command_interfaces(LoanedCommandInterfaces & loaned_command_interfaces)
{
  left_sprocket_spinning_joint_.register_command_interface(
        loaned_command_interfaces[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID]);
  right_sprocket_spinning_joint_.register_command_interface(
        loaned_command_interfaces[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::register_loaned_state_interfaces(LoanedStateInterfaces & loaned_state_interfaces)
{
  left_sprocket_spinning_joint_.register_state_interface(
        loaned_state_interfaces[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID]);
  right_sprocket_spinning_joint_.register_state_interface(
        loaned_state_interfaces[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID]);
}
//-----------------------------------------------------------------------------
void ControllerInterface2TD::set_command(const OdometryFrame2WD &command)
{
  left_sprocket_spinning_joint_.set_command(command.leftWheelSpeed);
  right_sprocket_spinning_joint_.set_command(command.rightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame2WD ControllerInterface2TD::get_odometry_frame() const
{
  OdometryFrame2WD odometry;
  odometry.leftWheelSpeed = left_sprocket_spinning_joint_.get_measurement();
  odometry.rightWheelSpeed = right_sprocket_spinning_joint_.get_measurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2TD::get_command_interface_names()const
{
  return {left_sprocket_spinning_joint_.get_command_interface_name(),
        right_sprocket_spinning_joint_.get_command_interface_name()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2TD::get_state_interface_names()const
{
  return {left_sprocket_spinning_joint_.get_state_interface_name(),
        right_sprocket_spinning_joint_.get_state_interface_name()};
}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::declare_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,left_sprocket_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,right_sprocket_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2TD::get_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node,parameters_ns,left_sprocket_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,right_sprocket_wheel_spinning_joint_param_name)};
}

}

