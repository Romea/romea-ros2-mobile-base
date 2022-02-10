#ifndef _romea_CommandConversions_hpp_
#define _romea_CommandConversions_hpp_

//romea
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp>

//ros
#include <geometry_msgs/msg/twist.hpp>
#include <four_wheel_steering_msgs/msg/four_wheel_steering.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <romea_mobile_base_msgs/msg/one_axle_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/two_axle_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/skid_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/omni_steering_command.hpp>

namespace romea {



void to_ros_msg(const romea::TwoAxleSteeringCommand & romea_two_axle_steering_command,
                four_wheel_steering_msgs::msg::FourWheelSteering & ros_four_wheel_steering_msg);

void to_ros_msg(const romea::OneAxleSteeringCommand & romea_one_axle_steering_command,
                ackermann_msgs::msg::AckermannDrive & ros_ackerman_drive_msg);

void to_ros_msg(const romea::OneAxleSteeringCommand & romea_one_axle_steering_command,
                geometry_msgs::msg::Twist & ros_twist_msg);

void to_ros_msg(const romea::SkidSteeringCommand & romea_skid_steering_command,
                geometry_msgs::msg::Twist & ros_twist_msg );

void to_ros_msg(const romea::OmniSteeringCommand & romea_omni_steeringcommand,
                geometry_msgs::msg::Twist & ros_twist_msg );


void to_ros_msg(const romea::TwoAxleSteeringCommand & romea_two_axle_steering_command,
                romea_mobile_base_msgs::msg::TwoAxleSteeringCommand & ros_two_axle_steering_command_msg);

void to_romea(const romea_mobile_base_msgs::msg::TwoAxleSteeringCommand & ros_two_axle_steering_command_msg,
             romea::TwoAxleSteeringCommand & romea_two_axle_steering_command);

void to_ros_msg(const romea::OneAxleSteeringCommand & romea_one_axle_steering_command,
                romea_mobile_base_msgs::msg::OneAxleSteeringCommand & ros_oxe_axle_steering_command_msg);

void to_romea(const romea_mobile_base_msgs::msg::OneAxleSteeringCommand & ros_one_axle_steering_command_msg,
             romea::OneAxleSteeringCommand & romea_one_axle_steering_command);

void to_ros_msg(const romea::SkidSteeringCommand & romea_skid_steering_command,
                romea_mobile_base_msgs::msg::SkidSteeringCommand & romea_skid_steering_command_msg );

void to_romea(const romea_mobile_base_msgs::msg::SkidSteeringCommand & ros_skid_steering_command_msg,
             romea::SkidSteeringCommand & romea_skid_steering_command);

void to_ros_msg(const romea::OmniSteeringCommand & romea_omni_steering_command,
                romea_mobile_base_msgs::msg::OmniSteeringCommand & romea_omni_steering_command_msg );

void to_romea(const romea_mobile_base_msgs::msg::OmniSteeringCommand & ros_omni_steering_command_msg,
             romea::OmniSteeringCommand & romea_omni_steering_command);


}// namespace
#endif
