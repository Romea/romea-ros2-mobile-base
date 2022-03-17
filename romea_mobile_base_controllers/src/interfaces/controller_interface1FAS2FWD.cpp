#include "romea_mobile_base_controllers/interfaces/controller_interface1FAS2FWD.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace {
const std::string front_axle_steering_joint_param_name="front_axle_steering_joint_name";
const std::string front_left_wheel_spinning_joint_param_name="front_left_wheel_spinning_joint_name";
const std::string front_right_wheel_spinning_joint_param_name="front_right_wheel_spinning_joint_name";
}

namespace romea
{

//-----------------------------------------------------------------------------
ControllerInterface1FAS2FWD::
ControllerInterface1FAS2FWD(const MobileBaseInfo1FAS2FWD & mobile_base_info,
                            const std::map<int,std::string> & joint_mappings,
                            LoanedCommandInterfaces & loaned_command_interfaces,
                            LoanedStateInterfaces & loaned_state_interfaces):
  front_steering_joint_(loaned_command_interfaces,
                        loaned_state_interfaces,
                        joint_mappings.at(FRONT_AXLE_STEERING_JOINT_ID)),
  front_left_spinning_joint_(loaned_command_interfaces,
                             loaned_state_interfaces,
                             joint_mappings.at(FRONT_LEFT_WHEEL_SPINNING_JOINT_ID),
                             mobile_base_info.geometry.rearAxle.wheels.radius),
  front_right_spinning_joint_(loaned_command_interfaces,
                              loaned_state_interfaces,
                              joint_mappings.at(FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID),
                              mobile_base_info.geometry.rearAxle.wheels.radius)
{
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2FWD::setCommand(const OdometryFrame1FAS2FWD &command)
{
  front_steering_joint_.setCommand(command.frontAxleSteeringAngle);
  front_left_spinning_joint_.setCommand(command.frontLeftWheelSpeed);
  front_right_spinning_joint_.setCommand(command.frontRightWheelSpeed);
}

//-----------------------------------------------------------------------------
OdometryFrame1FAS2FWD ControllerInterface1FAS2FWD::getOdometryFrame() const
{
  OdometryFrame1FAS2FWD odometry;
  odometry.frontAxleSteeringAngle = front_steering_joint_.getMeasurement();
  odometry.frontLeftWheelSpeed = front_left_spinning_joint_.getMeasurement();
  odometry.frontRightWheelSpeed = front_right_spinning_joint_.getMeasurement();
  return odometry;
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2FWD::getCommandInterfaceNames()const
{
  return  {  front_steering_joint_.getCommandInterfaceName(),
        front_left_spinning_joint_.getCommandInterfaceName(),
        front_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
std::vector<std::string> ControllerInterface1FAS2FWD::getStateInterfaceNames()const
{
  return  {  front_steering_joint_.getStateInterfaceName(),
        front_left_spinning_joint_.getStateInterfaceName(),
        front_right_spinning_joint_.getCommandInterfaceName()};
}

//-----------------------------------------------------------------------------
void ControllerInterface1FAS2FWD::declare_joints_mapping(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  declare_parameter<std::string>(node,parameters_ns,front_axle_steering_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name);
  declare_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name);
}

//-----------------------------------------------------------------------------
std::map<int,std::string> ControllerInterface1FAS2FWD::get_joints_mapping(
    std::shared_ptr<rclcpp::Node> node, const std::string & parameters_ns)
{
  std::map<int,std::string> joint_mappings;
  joint_mappings[FRONT_AXLE_STEERING_JOINT_ID]=
      get_parameter<std::string>(node,parameters_ns,front_axle_steering_joint_param_name);
  joint_mappings[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID]=
      get_parameter<std::string>(node,parameters_ns,front_left_wheel_spinning_joint_param_name);
  joint_mappings[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID]=
      get_parameter<std::string>(node,parameters_ns,front_right_wheel_spinning_joint_param_name);
  return joint_mappings;
}

}

