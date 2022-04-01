#include "romea_mobile_base_controllers/interfaces/controller_interface2TD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string left_sprocket_wheel_spinning_joint_param_name="left_sprocket_wheel_spinning_joint_name";
const std::string right_sprocket_wheel_spinning_joint_param_name="right_sprocket_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2TD::ControllerInterface2TD(const MobileBaseInfo2TD &mobile_base_info):
  spinning_joints_(mobile_base_info.geometry.tracks.sprocketWheel.radius+
                   mobile_base_info.geometry.tracks.thickness)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::write(const OdometryFrame2WD &command, LoanedCommandInterfaces & loaned_command_interfaces)const
{
  spinning_joints_.write(command.leftWheelSpeed,loaned_command_interfaces[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID]);
  spinning_joints_.write(command.rightWheelSpeed,loaned_command_interfaces[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface2TD::read(const LoanedStateInterfaces & loaned_state_interfaces, OdometryFrame2WD & measurement) const
{
  spinning_joints_.read(loaned_state_interfaces[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID],measurement.leftWheelSpeed);
  spinning_joints_.read(loaned_state_interfaces[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID],measurement.rightWheelSpeed);
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


//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2TD::hardware_interface_names(
    const std::vector<std::string> & joints_names)
{
  return {SpinningJointControllerInterface::hardware_interface_name(
          joints_names[LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID])};
}

}

