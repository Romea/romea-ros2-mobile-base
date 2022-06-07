//romea
#include "romea_mobile_base_controllers/dead_reckoning_publisher.hpp"
#include <romea_mobile_base_utils/conversions/kinematic_conversions.hpp>

//ros
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace
{
const std::string DEFAULT_BASE_FRAME_ID = "base_link";
const std::string DEFAULT_ODOM_FRAME_ID = "odom";
}

namespace romea
{


//-----------------------------------------------------------------------------
DeadReckoningPublisher::DeadReckoningPublisher(std::shared_ptr<rclcpp::Node> node,
                                               const std::string & odom_frame_id,
                                               const std::string & base_frame_id,
                                               const bool &enable_odom_tf):
    odom_pub_(nullptr),
    tf_odom_pub_(nullptr),
    dead_reckoning_()
{
  initOdomPublisher_(node,odom_frame_id,base_frame_id);

  if(enable_odom_tf)
  {
      initOdomTFPublisher_(node,odom_frame_id,base_frame_id);
  }

}

//-----------------------------------------------------------------------------
void DeadReckoningPublisher::initOdomPublisher_(std::shared_ptr<rclcpp::Node> node,
                                                const std::string & odom_frame_id,
                                                const std::string & base_frame_id)
{
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    odom_pub_ = std::make_shared<OdomPublisher>(odom_pub);
    odom_pub_->msg_.child_frame_id= base_frame_id;
    odom_pub_->msg_.header.frame_id=odom_frame_id;
}

//-----------------------------------------------------------------------------
void DeadReckoningPublisher::initOdomTFPublisher_(std::shared_ptr<rclcpp::Node> node,
                                                  const std::string & odom_frame_id,
                                                  const std::string & base_frame_id)
{
    auto tf_odom_pub = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
    tf_odom_pub_= std::make_shared<TfPublisher>(tf_odom_pub);
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id;
}


//-----------------------------------------------------------------------------
void DeadReckoningPublisher::update(const rclcpp::Time& time,
                                    const KinematicMeasure & kinematic_measure)
{

    dead_reckoning_.update(time,kinematic_measure);

    const tf2::Quaternion q(tf2::Vector3(0,0,1),dead_reckoning_.getHeading());
    const geometry_msgs::msg::Quaternion orientation= tf2::toMsg(q);

    // Populate odom message and publish
    if (odom_pub_->trylock())
    {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = dead_reckoning_.getX();
        odom_pub_->msg_.pose.pose.position.y = dead_reckoning_.getY();
        odom_pub_->msg_.pose.pose.orientation=orientation;
        to_ros_msg(kinematic_measure,odom_pub_->msg_.twist);
        odom_pub_->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (tf_odom_pub_!=nullptr && tf_odom_pub_->trylock())
    {
        geometry_msgs::msg::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = dead_reckoning_.getX();
        odom_frame.transform.translation.y = dead_reckoning_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
    }
}

//-----------------------------------------------------------------------------
void  DeadReckoningPublisher::reset()
{
    dead_reckoning_.reset();
}

}
