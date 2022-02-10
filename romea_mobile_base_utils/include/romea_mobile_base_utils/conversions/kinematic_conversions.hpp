#ifndef _romea_KinematicConversions_hpp_
#define _romea_KinematicConversions_hpp_

//romea
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp>

//ros
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <romea_mobile_base_msgs/msg/kinematic_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/one_axle_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/two_axle_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/skid_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/omni_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/one_axle_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/two_axle_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/skid_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/omni_steering_command.hpp>

namespace romea {


KinematicMeasure to_romea(const romea_mobile_base_msgs::msg::KinematicMeasure & msg);

void to_ros_msg(const KinematicMeasure & romea_kinematic_measure,
                romea_mobile_base_msgs::msg::KinematicMeasure  & ros_kinematic_msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const KinematicMeasure & romea_kinematic_measure,
                romea_mobile_base_msgs::msg::KinematicMeasureStamped & ros_kinematic_stamped_msg);

void to_ros_msg(const KinematicMeasure & romea_kinematic_measure,
                 geometry_msgs::msg::TwistWithCovariance & ros_twist_with_covariance);


OneAxleSteeringMeasure to_romea(const romea_mobile_base_msgs::msg::OneAxleSteeringMeasure & msg);

void to_ros_msg(const OneAxleSteeringMeasure & romea_one_axle_steering_measure,
                romea_mobile_base_msgs::msg::OneAxleSteeringMeasure  & ros_one_axle_steering_measure_msg);

void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const OneAxleSteeringMeasure & romea_one_axle_steering_measure,
                romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped & ros_one_axle_steering_measure_msg);



TwoAxleSteeringMeasure to_romea(const romea_mobile_base_msgs::msg::TwoAxleSteeringMeasure & msg);

void to_ros_msg(const TwoAxleSteeringMeasure & romea_two_axle_steering_measure,
                romea_mobile_base_msgs::msg::TwoAxleSteeringMeasure  & ros_two_axle_steering_measure_msg);


void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const TwoAxleSteeringMeasure & romea_two_axle_steering_measure,
                romea_mobile_base_msgs::msg::TwoAxleSteeringMeasureStamped & ros_two_axle_steering_measure_msg);


SkidSteeringMeasure to_romea(const romea_mobile_base_msgs::msg::SkidSteeringMeasure & msg);

void to_ros_msg(const SkidSteeringMeasure & romea_skid_steering_measure,
                romea_mobile_base_msgs::msg::SkidSteeringMeasure  & ros_skid_steering_measure_msg);


void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const SkidSteeringMeasure & romea_skid_steering_measure,
                romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped & ros_skid_steering_measure_msg);


OmniSteeringMeasure to_romea(const romea_mobile_base_msgs::msg::OmniSteeringMeasure & msg);

void to_ros_msg(const OmniSteeringMeasure & romea_omni_steering_measure,
                romea_mobile_base_msgs::msg::OmniSteeringMeasure  & ros_omni_steering_measure_msg);


void to_ros_msg(const rclcpp::Time & stamp,
                const std::string & frame_id,
                const OmniSteeringMeasure & romea_omni_steering_measure,
                romea_mobile_base_msgs::msg::OmniSteeringMeasureStamped & ros_omni_steering_measure_msg);

}// namespace
#endif
