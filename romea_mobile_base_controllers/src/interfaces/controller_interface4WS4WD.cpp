#include "romea_mobile_base_controllers/interfaces/controller_interface4WS4WD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string front_left_wheel_steering_joint_param_name="front_left_wheel_steering_joint_name";
const std::string front_right_wheel_steering_joint_param_name="front_right_wheel_steering_joint_name";
const std::string rear_left_wheel_steering_joint_param_name="rear_left_wheel_steering_joint_name";
const std::string rear_right_wheel_steering_joint_param_name="rear_right_wheel_steering_joint_name";
const std::string front_left_wheel_spinning_joint_param_name="front_left_wheel_spinning_joint_name";
const std::string front_right_wheel_spinning_joint_param_name="front_right_wheel_spinning_joint_name";
const std::string rear_left_wheel_spinning_joint_param_name="rear_left_wheel_spinning_joint_name";
const std::string rear_right_wheel_spinning_joint_param_name="rear_right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface4WS4WD::ControllerInterface4WS4WD(const MobileBaseInfo4WS4WD & mobile_base_info):
  steering_joints_(),
  front_spinning_joints_(mobile_base_info.geometry.frontAxle.wheels.radius),
  rear_spinning_joints_(mobile_base_info.geometry.frontAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface4WS4WD::write(const OdometryFrame4WS4WD & command,
                                       LoanedCommandInterfaces & loaned_command_interfaces)const
{
  steering_joints_.write(command.frontLeftWheelAngle,loaned_command_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]);
  steering_joints_.write(command.frontRightWheelAngle,loaned_command_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]);
  steering_joints_.write(command.rearLeftWheelAngle,loaned_command_interfaces[REAR_LEFT_WHEEL_STEERING_JOINT_ID]);
  steering_joints_.write(command.rearRightWheelAngle,loaned_command_interfaces[REAR_RIGHT_WHEEL_STEERING_JOINT_ID]);
  front_spinning_joints_.write(command.frontLeftWheelSpeed,loaned_command_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]);
  front_spinning_joints_.write(command.frontRightWheelSpeed,loaned_command_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]);
  rear_spinning_joints_.write(command.rearLeftWheelSpeed,loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]);
  rear_spinning_joints_.write(command.rearRightWheelSpeed,loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface4WS4WD::read(const LoanedStateInterfaces & loaned_state_interfaces,
                                      OdometryFrame4WS4WD & measurement)const
{
  steering_joints_.read(loaned_state_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID],measurement.frontLeftWheelAngle);
  steering_joints_.read(loaned_state_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID],measurement.frontRightWheelAngle);
  steering_joints_.read(loaned_state_interfaces[REAR_LEFT_WHEEL_STEERING_JOINT_ID],measurement.rearLeftWheelAngle);
  steering_joints_.read(loaned_state_interfaces[REAR_RIGHT_WHEEL_STEERING_JOINT_ID],measurement.rearRightWheelAngle);
  front_spinning_joints_.read(loaned_state_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],measurement.frontLeftWheelSpeed);
  front_spinning_joints_.read(loaned_state_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],measurement.frontRightWheelSpeed);
  rear_spinning_joints_.read(loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],measurement.rearLeftWheelSpeed);
  rear_spinning_joints_.read(loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],measurement.rearRightWheelSpeed);

}

//-----------------------------------------------------------------------------
void ControllerInterface4WS4WD::declare_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_right_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name);

}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WS4WD::get_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_left_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_right_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name)};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface4WS4WD::hardware_interface_names(
    const std::vector<std::string> & joints_names)
{
  return {SteeringJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
        SteeringJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
        SteeringJointControllerInterface::hardware_interface_name(
          joints_names[REAR_LEFT_WHEEL_STEERING_JOINT_ID]),
        SteeringJointControllerInterface::hardware_interface_name(
          joints_names[REAR_RIGHT_WHEEL_STEERING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]),
        SpinningJointControllerInterface::hardware_interface_name(
          joints_names[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}


