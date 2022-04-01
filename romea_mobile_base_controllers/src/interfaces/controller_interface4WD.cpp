#include "romea_mobile_base_controllers/interfaces/controller_interface4WD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string front_left_wheel_steering_joint_param_name="front_left_wheel_spinning_joint_name";
const std::string front_right_wheel_steering_joint_param_name="front_right_wheel_spinning_joint_name";
const std::string rear_left_wheel_steering_joint_param_name="rear_left_wheel_spinning_joint_name";
const std::string rear_right_wheel_steering_joint_param_name="rear_right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface4WD::ControllerInterface4WD(const MobileBaseInfo4WD & mobile_base_info):
  front_spinning_joints_(mobile_base_info.geometry.frontAxle.wheels.radius),
  rear_spinning_joints_(mobile_base_info.geometry.rearAxle.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface4WD::write(const OdometryFrame4WD &command, LoanedCommandInterfaces & loaned_command_interfaces)const
{
  front_spinning_joints_.write(command.frontLeftWheelSpeed,loaned_command_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]);
  front_spinning_joints_.write(command.frontRightWheelSpeed,loaned_command_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]);
  rear_spinning_joints_.write(command.rearLeftWheelSpeed,loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]);
  rear_spinning_joints_.write(command.rearRightWheelSpeed,loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID]);
}


//-----------------------------------------------------------------------------
void ControllerInterface4WD::read(const LoanedStateInterfaces & loaned_state_interfaces, OdometryFrame4WD & measurement) const
{
  front_spinning_joints_.read(loaned_state_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],measurement.frontLeftWheelSpeed);
  front_spinning_joints_.read(loaned_state_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],measurement.frontRightWheelSpeed);
  rear_spinning_joints_.read(loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],measurement.rearLeftWheelSpeed);
  rear_spinning_joints_.read(loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],measurement.rearRightWheelSpeed);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WD::hardware_interface_names(
    const std::vector<std::string> & joints_names)
{
  return {SpinningJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID])
  };
}

//-----------------------------------------------------------------------------
void ControllerInterface4WD::declare_joints_names(std::shared_ptr<rclcpp::Node> node, const std::string &parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_right_wheel_steering_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WD::get_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_left_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_right_wheel_steering_joint_param_name)};
}


}
