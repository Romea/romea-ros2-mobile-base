#include "romea_mobile_base_utils/params/command_limits_parameters.hpp"
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_common/math/Algorithm.hpp>

namespace  {

const double DEFAULT_MINIMAL_SPEED=0.;
const double DEFAULT_MAXIMAL_SPEED=std::numeric_limits<double>::max();
const double DEFAULT_MAXIMAL_ANGULAR_SPEED=std::numeric_limits<double>::max();
const double DEFAULT_MAXIMAL_STEERGING_ANGLE= M_PI_2;

const std::string MINIMAL_LONGITUDINAL_SPEED_PARAM_NAME = "minimal_longitudinal_speed";
const std::string MAXIMAL_LONGITUDINAL_SPEED_PARAM_NAME = "maximal_longitudinal_speed";
const std::string MAXIMAL_LATERAL_SPEED_PARAM_NAME = "maximal_lateral_speed";
const std::string MAXIMAL_ANGULAR_SPEED_PARAM_NAME = "maximal_angular_speed";

const std::string MAXIMAL_STEERING_ANGLE_PARAM_NAME = "maximal_steering_angle";
const std::string MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME = "maximal_front_steering_angle";
const std::string MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME = "maximal_rear_steering_angle";


void declare_longitudinal_speed_command_limits(std::shared_ptr<rclcpp::Node> node,
                                               const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(node,parameters_ns,
                                                MINIMAL_LONGITUDINAL_SPEED_PARAM_NAME,
                                                DEFAULT_MINIMAL_SPEED);

  romea::declare_parameter_with_default<double>(node,parameters_ns,
                                                MAXIMAL_LONGITUDINAL_SPEED_PARAM_NAME,
                                                DEFAULT_MAXIMAL_SPEED);
}

void declare_lateral_speed_command_limits(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(node,parameters_ns,
                                                MAXIMAL_LATERAL_SPEED_PARAM_NAME,
                                                DEFAULT_MAXIMAL_SPEED);

}

void declare_angular_speed_command_limits(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(node,parameters_ns,
                                                MAXIMAL_ANGULAR_SPEED_PARAM_NAME,
                                                DEFAULT_MAXIMAL_ANGULAR_SPEED);
}

void declare_steering_angle_command_limits(std::shared_ptr<rclcpp::Node> node,
                                           const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(node,parameters_ns,
                                                MAXIMAL_STEERING_ANGLE_PARAM_NAME,
                                                DEFAULT_MAXIMAL_STEERGING_ANGLE);
}

void declare_front_steering_angle_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(node,parameters_ns,
                                                MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME,
                                                DEFAULT_MAXIMAL_STEERGING_ANGLE);
}

void declare_rear_steering_angle_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                const std::string & parameters_ns)
{
  romea::declare_parameter_with_default<double>(node,parameters_ns,
                                                MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME,
                                                DEFAULT_MAXIMAL_STEERGING_ANGLE);
}


romea::Interval1D<double> get_longitudinal_speed_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                                const std::string & parameters_ns)
{
  return romea::makeLongitudinalSpeedCommandLimits(
        romea::get_parameter<double>(node,parameters_ns,MINIMAL_LONGITUDINAL_SPEED_PARAM_NAME),
        romea::get_parameter<double>(node,parameters_ns,MAXIMAL_LONGITUDINAL_SPEED_PARAM_NAME));
}

romea::Interval1D<double> get_lateral_speed_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                           const std::string & parameters_ns)
{
  return romea::makeSymmetricCommandLimits(
        romea::get_parameter<double>(node,parameters_ns,MAXIMAL_LATERAL_SPEED_PARAM_NAME));
}

romea::Interval1D<double> get_angular_speed_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                           const std::string & parameters_ns)
{
  return romea::makeSymmetricCommandLimits(
        romea::get_parameter<double>(node,parameters_ns,MAXIMAL_ANGULAR_SPEED_PARAM_NAME));
}

romea::Interval1D<double> get_steering_angle_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                            const std::string & parameters_ns)
{
  return romea::makeSymmetricCommandLimits(
        romea::get_parameter<double>(node,parameters_ns,MAXIMAL_STEERING_ANGLE_PARAM_NAME));
}

romea::Interval1D<double> get_front_steering_angle_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                                  const std::string & parameters_ns)
{
  return romea::makeSymmetricCommandLimits(
        romea::get_parameter<double>(node,parameters_ns,MAXIMAL_FRONT_STEERING_ANGLE_PARAM_NAME));
}

romea::Interval1D<double> get_rear_steering_angle_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                                 const std::string & parameters_ns)
{
  return romea::makeSymmetricCommandLimits(
        romea::get_parameter<double>(node,parameters_ns,MAXIMAL_REAR_STEERING_ANGLE_PARAM_NAME));
}

}

namespace romea
{

//-----------------------------------------------------------------------------
void declare_one_axle_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & parameters_ns)
{
  declare_longitudinal_speed_command_limits(node,parameters_ns);
  declare_steering_angle_command_limits(node,parameters_ns);
}

//-----------------------------------------------------------------------------
void declare_skid_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  declare_longitudinal_speed_command_limits(node,parameters_ns);
  declare_angular_speed_command_limits(node,parameters_ns);
}

//-----------------------------------------------------------------------------
void declare_two_axle_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & parameters_ns)
{
  declare_longitudinal_speed_command_limits(node,parameters_ns);
  declare_front_steering_angle_command_limits(node,parameters_ns);
  declare_rear_steering_angle_command_limits(node,parameters_ns);
}

//-----------------------------------------------------------------------------
void declare_omni_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  declare_longitudinal_speed_command_limits(node,parameters_ns);
  declare_lateral_speed_command_limits(node,parameters_ns);
  declare_angular_speed_command_limits(node,parameters_ns);
}


//-----------------------------------------------------------------------------
void get_one_axle_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns,
                                          OneAxleSteeringCommandLimits &limits)
{
  limits.longitudinalSpeed = get_longitudinal_speed_command_limits(node,parameters_ns);
  limits.steeringAngle = get_steering_angle_command_limits(node,parameters_ns);

}

//-----------------------------------------------------------------------------
void get_two_axle_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns,
                                          TwoAxleSteeringCommandLimits &limits)
{
  limits.longitudinalSpeed = get_longitudinal_speed_command_limits(node,parameters_ns);
  limits.frontSteeringAngle = get_front_steering_angle_command_limits(node,parameters_ns);
  limits.rearSteeringAngle = get_rear_steering_angle_command_limits(node,parameters_ns);

}

//-----------------------------------------------------------------------------
void get_skid_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                      const std::string & parameters_ns,
                                      SkidSteeringCommandLimits &limits)
{
  limits.longitudinalSpeed = get_longitudinal_speed_command_limits(node,parameters_ns);
  limits.angularSpeed = get_angular_speed_command_limits(node,parameters_ns);
}

//-----------------------------------------------------------------------------
void get_omni_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                      const std::string & parameters_ns,
                                      OmniSteeringCommandLimits &limits)
{
  limits.longitudinalSpeed = get_longitudinal_speed_command_limits(node,parameters_ns);
  limits.lateralSpeed = get_lateral_speed_command_limits(node,parameters_ns);
  limits.angularSpeed = get_angular_speed_command_limits(node,parameters_ns);
}


}


