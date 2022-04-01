#include "romea_mobile_base_controllers/interfaces/controller_interface2AS4WD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string front_axle_steering_joint_param_name="front_axle_steering_joint_name";
const std::string rear_axle_steering_joint_param_name="rear_axle_steering_joint_name";
const std::string front_left_wheel_spinning_joint_param_name="front_left_wheel_spinning_joint_name";
const std::string front_right_wheel_spinning_joint_param_name="front_right_wheel_spinning_joint_name";
const std::string rear_left_wheel_spinning_joint_param_name="rear_left_wheel_spinning_joint_name";
const std::string rear_right_wheel_spinning_joint_param_name="rear_right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2AS4WD::
ControllerInterface2AS4WD(const MobileBaseInfo2AS4WD & mobile_base_info):
  steering_joints_(),
  front_spinning_joints_(mobile_base_info.geometry.frontAxle.wheels.radius),
  rear_spinning_joints_(mobile_base_info.geometry.rearAxle.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface2AS4WD::write(const OdometryFrame2AS4WD & command,
                                       LoanedCommandInterfaces & loaned_command_interfaces)const
{
  steering_joints_.write(command.frontAxleSteeringAngle, loaned_command_interfaces[FRONT_AXLE_STEERING_JOINT_ID]);
  steering_joints_.write(command.rearAxleSteeringAngle,loaned_command_interfaces[REAR_AXLE_STEERING_JOINT_ID]);
  front_spinning_joints_.write(command.frontLeftWheelSpeed,loaned_command_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]);
  front_spinning_joints_.write(command.frontRightWheelSpeed,loaned_command_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]);
  rear_spinning_joints_.write(command.rearLeftWheelSpeed,loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]);
  rear_spinning_joints_.write(command.rearRightWheelSpeed,loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface2AS4WD::read(const LoanedStateInterfaces & loaned_state_interfaces,
                                      OdometryFrame2AS4WD & measurement)const
{
  steering_joints_.read(loaned_state_interfaces[FRONT_AXLE_STEERING_JOINT_ID],measurement.frontAxleSteeringAngle);
  steering_joints_.read(loaned_state_interfaces[REAR_AXLE_STEERING_JOINT_ID],measurement.rearAxleSteeringAngle);
  front_spinning_joints_.read(loaned_state_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],measurement.frontLeftWheelSpeed);
  front_spinning_joints_.read(loaned_state_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],measurement.frontRightWheelSpeed);
  rear_spinning_joints_.read(loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],measurement.rearLeftWheelSpeed);
  rear_spinning_joints_.read(loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],measurement.rearRightWheelSpeed);
}


//-----------------------------------------------------------------------------
void ControllerInterface2AS4WD::declare_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,front_axle_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_axle_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2AS4WD::get_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node,parameters_ns,front_axle_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_axle_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name)};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2AS4WD::hardware_interface_names(
    const std::vector<std::string> & joints_names)
{
  return {SteeringJointControllerInterface::hardware_interface_name(
          joints_names[FRONT_AXLE_STEERING_JOINT_ID]),
        SteeringJointControllerInterface::hardware_interface_name(
          joints_names[REAR_AXLE_STEERING_JOINT_ID]),
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

