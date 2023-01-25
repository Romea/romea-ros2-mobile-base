// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_PUBLISHER_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_PUBLISHER_HPP_

// std
#include <memory>
#include <string>

// romea ros
#include "romea_mobile_base_msgs/msg/kinematic_measure_stamped.hpp"

// ros
#include "rclcpp/node.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"


// local
#include "dead_reckoning.hpp"

namespace romea
{

class DeadReckoningPublisher
{
private:
  using TfPublisher = realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>;
  using OdomPublisher = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
  using KinematicPublisher =
    realtime_tools::RealtimePublisher<romea_mobile_base_msgs::msg::KinematicMeasureStamped>;

public:
  DeadReckoningPublisher(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & odom_frame_id,
    const std::string & base_frame_id,
    const bool & enable_odom_tf);

  void update(
    const rclcpp::Time & time,
    const KinematicMeasure & kinematic_measure);

  void reset();

private:
  void initOdomPublisher_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & odom_frame_id,
    const std::string & base_frame_id);

  void initOdomTFPublisher_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & odom_frame_id,
    const std::string & base_frame_id);

private:
  std::shared_ptr<OdomPublisher> odom_pub_;
  std::shared_ptr<TfPublisher> tf_odom_pub_;
  DeadReckoning dead_reckoning_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_PUBLISHER_HPP_
