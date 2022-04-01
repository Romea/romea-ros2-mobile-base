#include "romea_mobile_base_controllers/interfaces/controller_interface2FWS2FWD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string front_left_wheel_steering_joint_param_name="front_left_wheel_steering_joint_name";
const std::string front_right_wheel_steering_joint_param_name="front_right_wheel_steering_joint_name";
const std::string front_left_wheel_spinning_joint_param_name="front_left_wheel_spinning_joint_name";
const std::string front_right_wheel_spinning_joint_param_name="front_right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2FWS2FWD::ControllerInterface2FWS2FWD(const MobileBaseInfo2FWS2FWD & mobile_base_info):
  front_steering_joints_(),
  front_spinning_joints_(mobile_base_info.geometry.frontAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2FWD::write(const OdometryFrame2FWS2FWD & command,
                                        LoanedCommandInterfaces & loaned_command_interfaces)const
{
  front_steering_joints_.write(command.frontLeftWheelAngle,loaned_command_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]);
  front_steering_joints_.write(command.frontRightWheelAngle,loaned_command_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]);
  front_spinning_joints_.write(command.frontLeftWheelSpeed,loaned_command_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]);
  front_spinning_joints_.write(command.frontRightWheelSpeed,loaned_command_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2FWD::read(const LoanedStateInterfaces & loaned_state_interfaces,
                                       OdometryFrame2FWS2FWD & measurement)const
{
  front_steering_joints_.read(loaned_state_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID],measurement.frontLeftWheelAngle);
  front_steering_joints_.read(loaned_state_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID],measurement.frontRightWheelAngle);
  front_spinning_joints_.read(loaned_state_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],measurement.frontLeftWheelSpeed);
  front_spinning_joints_.read(loaned_state_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],measurement.frontRightWheelSpeed);
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2FWD::declare_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2FWS2FWD::get_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name)};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2FWS2FWD::hardware_interface_names(
    const std::vector<std::string> & joints_names)
{
  return {SteeringJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
        SteeringJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}

