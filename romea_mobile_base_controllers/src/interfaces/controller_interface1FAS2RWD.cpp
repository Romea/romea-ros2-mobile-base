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
ControllerInterface1FAS2RWD(const MobileBaseInfo1FAS2RWD & mobile_base_info):
  rear_wheels_radius_(mobile_base_info.geometry.rearAxle.wheels.radius)
{

}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::write(const OdometryFrame1FAS2RWD & command,
                                        LoanedCommandInterfaces & loaned_command_interfaces)const
{
  loaned_command_interfaces[FRONT_AXLE_STEERING_JOINT_ID].
      set_value(command.frontAxleSteeringAngle);
  loaned_command_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].
      set_value(command.rearLeftWheelLinearSpeed/rear_wheels_radius_);
  loaned_command_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].
      set_value(command.rearRightWheelLinearSpeed/rear_wheels_radius_);
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2RWD::read(const LoanedStateInterfaces & loaned_state_interfaces,
                                       OdometryFrame1FAS2RWD & measurement)const
{
  measurement.frontAxleSteeringAngle =
      loaned_state_interfaces[FRONT_AXLE_STEERING_JOINT_ID].get_value();
  measurement.rearLeftWheelLinearSpeed = rear_wheels_radius_ *
      loaned_state_interfaces[REAR_LEFT_WHEEL_SPINNING_JOINT_ID].get_value();
  measurement.rearRightWheelLinearSpeed = rear_wheels_radius_*
      loaned_state_interfaces[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID].get_value();
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

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2RWD::hardware_interface_names(
    const std::vector<std::string> & joints_names)
{
  return {hardware_position_interface_name(joints_names[FRONT_AXLE_STEERING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[REAR_LEFT_WHEEL_SPINNING_JOINT_ID]),
        hardware_velocity_interface_name(joints_names[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID])};
}

}

