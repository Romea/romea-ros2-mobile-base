//romea
#include "romea_odo_utils/command_publisher.hpp"
#include <romea_common_utils/publishers/message_publisher.hpp>

namespace romea {

//-----------------------------------------------------------------------------
template<class CommandType>
CommandPublisher<CommandType>::CommandPublisher()
//  publisher_topic_(),
{

}

//-----------------------------------------------------------------------------
template<class CommandType>
void CommandPublisher<CommandType>::publish(const CommandType & command)
{
  publisher_->publish(command);
}

////-----------------------------------------------------------------------------
//template<class CommandType>
//const std::string &CommandPublisher<CommandType>::getTopic() const
//{
//  return publisher_topic_;
//}

//-----------------------------------------------------------------------------
template<>
void CommandPublisher<SkidSteeringCommand>::init(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string & controller_type)
{
  if(controller_type == "diff_drive_controller/DiffDriveController")
  {
    make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
  }
  else
  {
    make_publisher_<romea_odo_msgs::msg::SkidSteeringCommand>(node,"cmd_skid_steering",1);
  }
}

//-----------------------------------------------------------------------------
template<>
void CommandPublisher<OneAxleSteeringCommand>::init(std::shared_ptr<rclcpp::Node> node,
                                                    const std::string & controller_type)
{
  if(controller_type == "ackermann_steering_controller/AckermannSteeringController" ||
     controller_type == "minimal_ackermann_controller/MinimalAckermannController")
  {
    make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
  }
  else
  {
    make_publisher_<romea_odo_msgs::msg::OneAxleSteeringCommand>(node,"cmd_one_axle_steeering",1);
  }
}

//-----------------------------------------------------------------------------
template<>
void CommandPublisher<OmniSteeringCommand>::init(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string & controller_type)
{
  if(controller_type == "campero_controller/CamperoController")
  {
    make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
  }
  else
  {
    make_publisher_<romea_odo_msgs::msg::OmniSteeringCommand>(node,"cmd_omni_steeering",1);
  }
}


//-----------------------------------------------------------------------------
template<>
void CommandPublisher<TwoAxleSteeringCommand>::init(std::shared_ptr<rclcpp::Node> node,
                                                    const std::string & controller_type)
{
  if(controller_type == "four_wheel_steering_controller/FourWheelSteeringController")
  {
    make_publisher_<four_wheel_steering_msgs::msg::FourWheelSteering>(node,"cmd_four_wheel_steering",1);
  }
  else
  {
    make_publisher_<romea_odo_msgs::msg::TwoAxleSteeringCommand>(node,"cmd_two_axle_steering",1);
  }
}


template class CommandPublisher<SkidSteeringCommand>;
template class CommandPublisher<OmniSteeringCommand>;
template class CommandPublisher<OneAxleSteeringCommand>;
template class CommandPublisher<TwoAxleSteeringCommand>;

}

