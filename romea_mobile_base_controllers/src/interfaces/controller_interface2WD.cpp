#include "romea_mobile_base_controllers/interfaces/controller_interface2WD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string left_wheel_spinning_joint_param_name="left_wheel_spinning_joint_name";
const std::string right_wheel_spinning_joint_param_name="right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2WD::ControllerInterface2WD(const MobileBaseInfo2WD & mobile_base_info,
                                               const std::vector<std::string> &joints_names):
  left_spinning_joint_(joints_names[LEFT_WHEEL_SPINNING_JOINT_ID],
                       mobile_base_info.geometry.wheels.radius),
  right_spinning_joint_(joints_names[RIGHT_WHEEL_SPINNING_JOINT_ID],
                        mobile_base_info.geometry.wheels.radius)
{

}
//-----------------------------------------------------------------------------
void ControllerInterface2WD::register_loaned_command_interfaces(LoanedCommandInterfaces & loaned_command_interfaces)
{
  left_spinning_joint_.register_command_interface(
        loaned_command_interfaces[LEFT_WHEEL_SPINNING_JOINT_ID]);
  right_spinning_joint_.register_command_interface(
        loaned_command_interfaces[RIGHT_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::register_loaned_state_interfaces(LoanedStateInterfaces & loaned_state_interfaces)
{
  left_spinning_joint_.register_state_interface(
        loaned_state_interfaces[LEFT_WHEEL_SPINNING_JOINT_ID]);
  right_spinning_joint_.register_state_interface(
        loaned_state_interfaces[RIGHT_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::set_command(const OdometryFrame2WD &command)
{
  left_spinning_joint_.set_command(command.leftWheelSpeed);
  right_spinning_joint_.set_command(command.rightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame2WD ControllerInterface2WD::get_odometry_frame() const
{
  OdometryFrame2WD odometry;
  odometry.leftWheelSpeed = left_spinning_joint_.get_measurement();
  odometry.rightWheelSpeed = right_spinning_joint_.get_measurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2WD::get_command_interface_names()const
{
  return {left_spinning_joint_.get_command_interface_name(),
        right_spinning_joint_.get_command_interface_name()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2WD::get_state_interface_names()const
{
  return {left_spinning_joint_.get_state_interface_name(),
        right_spinning_joint_.get_state_interface_name()};
}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::declare_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2WD::get_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node,parameters_ns,left_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,right_wheel_spinning_joint_param_name)};
}

}

