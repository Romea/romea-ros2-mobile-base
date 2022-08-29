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
  front_wheels_radius_(mobile_base_info.geometry.frontAxle.wheels.radius),
  rear_wheels_radius_(mobile_base_info.geometry.rearAxle.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface2AS4WD::write(const OdometryFrame2AS4WD & command,
                                      LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[FRONT_AXLE_STEERING_JOINT_ID].
      set_value(command.frontAxleSteeringAngle);
  loaned_command_interfaces[REAR_AXLE_STEERING_JOINT_ID].
      set_value(command.rearAxleSteeringAngle);
  loaned_command_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID].
      set_value(command.frontLeftWheelLinearSpeed/front_wheels_radius_);
  loaned_command_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID].
      set_value(command.frontRightWheelLinearSpeed/front_wheels_radius_);
  loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].
      set_value(command.rearLeftWheelLinearSpeed/rear_wheels_radius_);
  loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].
      set_value(command.rearRightWheelLinearSpeed/rear_wheels_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface2AS4WD::read(const LoanedStateInterfaces & loaned_state_interfaces,
                                     OdometryFrame2AS4WD & measurement)const
{
  measurement.frontAxleSteeringAngle =
      loaned_state_interfaces[FRONT_AXLE_STEERING_JOINT_ID].get_value();
  measurement.rearAxleSteeringAngle =
      loaned_state_interfaces[REAR_AXLE_STEERING_JOINT_ID].get_value();
  measurement.frontLeftWheelLinearSpeed = front_wheels_radius_ *
      loaned_state_interfaces[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.frontRightWheelLinearSpeed = front_wheels_radius_ *
      loaned_state_interfaces[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rearLeftWheelLinearSpeed = rear_wheels_radius_ *
      loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rearRightWheelLinearSpeed = rear_wheels_radius_ *
      loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
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
  return {hardware_position_interface_name(joints_names[FRONT_AXLE_STEERING_JOINT_ID]),
        hardware_position_interface_name(joints_names[REAR_AXLE_STEERING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID])};
}


}

