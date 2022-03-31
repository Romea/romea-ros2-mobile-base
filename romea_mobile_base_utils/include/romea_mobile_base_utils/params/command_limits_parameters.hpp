#ifndef _romea_CommandConstraintsParameters_hpp_
#define _romea_CommandConstraintsParameters_hpp_

//romea
#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommandLimits.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommandLimits.hpp>

namespace romea {


void declare_one_axle_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & parameters_ns);

void declare_two_axle_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & parameters_ns);

void declare_omni_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns);

void declare_skid_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns);


SkidSteeringCommandLimits get_skid_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                           const std::string & parameters_ns);

OmniSteeringCommandLimits get_omni_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                           const std::string & parameters_ns);

OneAxleSteeringCommandLimits get_one_axle_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                                  const std::string & parameters_ns);

TwoAxleSteeringCommandLimits get_two_axle_steering_command_limits(std::shared_ptr<rclcpp::Node> node,
                                                                  const std::string & parameters_ns);


template <typename Limits>
void declare_command_limits(std::shared_ptr<rclcpp::Node> node,
                            const std::string & parameters_ns)
{
  if constexpr(std::is_same_v<Limits,SkidSteeringCommandLimits>)
  {
      declare_skid_steering_command_limits(node,parameters_ns);
  }
  else if constexpr (std::is_same_v<Limits,OmniSteeringCommandLimits>)
  {
      declare_omni_steering_command_limits(node,parameters_ns);
  }
  else if constexpr (std::is_same_v<Limits,OneAxleSteeringCommandLimits>)
  {
      declare_one_axle_steering_command_limits(node,parameters_ns);
  }
  else if constexpr (std::is_same_v<Limits,TwoAxleSteeringCommandLimits>)
  {
      declare_two_axle_steering_command_limits(node,parameters_ns);
  }
}


template <typename Limits>
Limits get_command_limits(std::shared_ptr<rclcpp::Node> node,
                          const std::string & parameters_ns)
{
  if constexpr ( std::is_same_v<Limits,SkidSteeringCommandLimits>)
      return get_skid_steering_command_limits(node,parameters_ns);
  else if constexpr (std::is_same_v<Limits,OmniSteeringCommandLimits>)
      return get_omni_steering_command_limits(node,parameters_ns);
  else if constexpr (std::is_same_v<Limits,OneAxleSteeringCommandLimits>)
      return get_one_axle_steering_command_limits(node,parameters_ns);
  else if constexpr (std::is_same_v<Limits,TwoAxleSteeringCommandLimits>)
      return get_two_axle_steering_command_limits(node,parameters_ns);
}


}

#endif
