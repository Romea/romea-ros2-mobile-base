#include "romea_mobile_base_controllers/interfaces/controller_interface2FWS2RWD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const std::string front_left_wheel_steering_joint_param_name="front_left_wheel_steering_joint_name";
const std::string front_right_wheel_steering_joint_param_name="front_right_wheel_steering_joint_name";
const std::string rear_left_wheel_spinning_joint_param_name="rear_left_wheel_spinning_joint_name";
const std::string rear_right_wheel_spinning_joint_param_name="rear_right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface2FWS2RWD::ControllerInterface2FWS2RWD(const MobileBaseInfo2FWS2RWD & mobile_base_info):
  rear_wheel_radius_(mobile_base_info.geometry.rearAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2RWD::write(const OdometryFrame2FWS2RWD & command,
                                        LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]
      .set_value(command.frontLeftWheelSteeringAngle);
  loaned_command_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]
      .set_value(command.frontRightWheelSteeringAngle);
  loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].
      set_value(command.rearLeftWheelLinearSpeed/rear_wheel_radius_);
  loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].
      set_value(command.rearRightWheelLinearSpeed/rear_wheel_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2RWD::read(const LoanedStateInterfaces & loaned_state_interfaces,
                                       OdometryFrame2FWS2RWD & measurement)const
{
  measurement.frontLeftWheelSteeringAngle =
      loaned_state_interfaces[FRONT_LEFT_WHEEL_STEERING_JOINT_ID].get_value();
  measurement.frontRightWheelSteeringAngle =
      loaned_state_interfaces[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID].get_value();
  measurement.rearLeftWheelLinearSpeed = rear_wheel_radius_ *
      loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rearRightWheelLinearSpeed = rear_wheel_radius_ *
      loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
}

//-----------------------------------------------------------------------------
void ControllerInterface2FWS2RWD::declare_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2FWS2RWD::get_joints_names(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  return {get_parameter<std::string>(node,parameters_ns,front_left_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,front_right_wheel_steering_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_left_wheel_spinning_joint_param_name),
        get_parameter<std::string>(node,parameters_ns,rear_right_wheel_spinning_joint_param_name)};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface2FWS2RWD::hardware_interface_names(
    const std::vector<std::string> & joints_names)
{
  return {hardware_position_interface_name(joints_names[FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
        hardware_position_interface_name(joints_names[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}

