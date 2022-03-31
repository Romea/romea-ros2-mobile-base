#include "romea_mobile_base_controllers/interfaces/controller_interface1FAS2RWD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string front_axle_steering_joint_param_name="front_axle_steering_joint_name";
const std::string rear_left_wheel_spinning_joint_param_name="rear_left_wheel_spinning_joint_name";
const std::string rear_right_wheel_spinning_joint_param_name="rear_right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface1FAS2RWD::
ControllerInterface1FAS2RWD(const MobileBaseInfo1FAS2RWD & mobile_base_info,
                            const std::vector<std::string> & joints_names):
  front_steering_joint_(joints_names[FRONT_AXLE_STEERING_JOINT_ID]),
  rear_left_spinning_joint_(joints_names[REAR_LEFT_WHEEL_SPINNING_JOINT_ID],
                            mobile_base_info.geometry.rearAxle.wheels.radius),
  rear_right_spinning_joint_(joints_names[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],
                             mobile_base_info.geometry.rearAxle.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::register_loaned_command_interfaces(LoanedCommandInterfaces & loaned_command_interfaces)
{
  front_steering_joint_.register_command_interface(
        loaned_command_interfaces[FRONT_AXLE_STEERING_JOINT_ID]);

  rear_left_spinning_joint_.register_command_interface(
        loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]);
  rear_right_spinning_joint_.register_command_interface(
        loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::register_loaned_state_interfaces(LoanedStateInterfaces & loaned_state_interfaces)
{
  front_steering_joint_.register_state_interface(
        loaned_state_interfaces[FRONT_AXLE_STEERING_JOINT_ID]);

  rear_left_spinning_joint_.register_state_interface(
        loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]);
  rear_right_spinning_joint_.register_state_interface(
        loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID]);
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::set_command(const OdometryFrame1FAS2RWD &command)
{
  front_steering_joint_.set_command(command.frontAxleSteeringAngle);
  rear_left_spinning_joint_.set_command(command.rearLeftWheelSpeed);
  rear_right_spinning_joint_.set_command(command.rearRightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame1FAS2RWD ControllerInterface1FAS2RWD::get_odometry_frame() const
{
  OdometryFrame1FAS2RWD odometry;
  odometry.frontAxleSteeringAngle = front_steering_joint_.get_measurement();
  odometry.rearLeftWheelSpeed = rear_left_spinning_joint_.get_measurement();
  odometry.rearRightWheelSpeed = rear_right_spinning_joint_.get_measurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2RWD::get_command_interface_names()const
{
  return {front_steering_joint_.get_command_interface_name(),
        rear_left_spinning_joint_.get_command_interface_name(),
        rear_right_spinning_joint_.get_command_interface_name()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2RWD::get_state_interface_names()const
{
  return {front_steering_joint_.get_state_interface_name(),
        rear_left_spinning_joint_.get_state_interface_name(),
        rear_right_spinning_joint_.get_state_interface_name()};
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::declare_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,front_axle_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2RWD::get_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return{get_parameter<std::string>(node,parameters_ns,front_axle_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name)};
}

}

