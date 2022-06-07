#ifndef DeadReckoningPublisher_H
#define DeadReckoningPublisher_H

//romea
#include "dead_reckoning.hpp"
#include <romea_mobile_base_msgs/msg/kinematic_measure_stamped.hpp>

//ros
#include <rclcpp/node.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace romea{

class DeadReckoningPublisher
{

private :

  using TfPublisher = realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>;
  using OdomPublisher = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
  using KinematicPublisher = realtime_tools::RealtimePublisher<romea_mobile_base_msgs::msg::KinematicMeasureStamped>;

public:

  DeadReckoningPublisher(std::shared_ptr<rclcpp::Node> node,
                         const std::string & odom_frame_id,
                         const std::string & base_frame_id,
                         const bool &enable_odom_tf);

  void update(const rclcpp::Time& time,
              const KinematicMeasure & kinematic_measure);

  void reset();

private :

  void initOdomPublisher_(std::shared_ptr<rclcpp::Node> node,
                          const std::string & odom_frame_id,
                          const std::string & base_frame_id);

  void initOdomTFPublisher_(std::shared_ptr<rclcpp::Node> node,
                            const std::string & odom_frame_id,
                            const std::string & base_frame_id);

private:

  std::shared_ptr<OdomPublisher> odom_pub_;
  std::shared_ptr<TfPublisher> tf_odom_pub_;
  DeadReckoning dead_reckoning_;
};

}

#endif
