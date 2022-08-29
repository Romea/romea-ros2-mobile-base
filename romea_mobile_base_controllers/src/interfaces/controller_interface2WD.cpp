#include "romea_mobile_base_controllers/interfaces/controller_interface2WD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string left_wheel_spinning_joint_param_name="left_wheel_spinning_joint_name";
const std::string right_wheel_spinning_joint_param_name="right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2WD::ControllerInterface2WD(const MobileBaseInfo2WD & mobile_base_info):
  wheels_radius_(mobile_base_info.geometry.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::write(const OdometryFrame2WD &command, LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[LEFT_WHEEL_SPINNING_JOINT_ID].set_value(
        command.leftWheelLinearSpeed/wheels_radius_);
  loaned_command_interfaces[RIGHT_WHEEL_SPINNING_JOINT_ID].set_value(
        command.rightWheelLinearSpeed/wheels_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface2WD::read(const LoanedStateInterfaces & loaned_state_interfaces, OdometryFrame2WD & measurement) const
{
  measurement.leftWheelLinearSpeed = wheels_radius_*
      loaned_state_interfaces[LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rightWheelLinearSpeed = wheels_radius_*
      loaned_state_interfaces[RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
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

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2WD::hardware_interface_names(
    const std::vector<std::string> & joints_names)
{
  return {hardware_velocity_interface_name(joints_names[LEFT_WHEEL_SPINNING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}
