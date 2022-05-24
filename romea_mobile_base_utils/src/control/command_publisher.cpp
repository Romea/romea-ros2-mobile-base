//romea
#include "romea_mobile_base_utils/control/command_publisher.hpp"
#include <romea_common_utils/publishers/data_publisher.hpp>

//std
#include <sstream>

namespace
{

template<class CommandType>
std::string command_type_name()
{
  if constexpr(std::is_same_v<CommandType,romea::SkidSteeringCommand>)
  {
    return "skid steering";
  }
  else if constexpr(std::is_same_v<CommandType,romea::OmniSteeringCommand>)
  {
    return "omni steering";
  }
  else if constexpr(std::is_same_v<CommandType,romea::OneAxleSteeringCommand>)
  {
    return "one axle steering";
  }
  else if constexpr(std::is_same_v<CommandType,romea::TwoAxleSteeringCommand>)
  {
    return "two axle steering";
  }
}

template<class CommandType>
void throw_exception(const std::string & message_type)
{
  std::stringstream msg;
  msg<< "Output message type ";
  msg<< message_type;
  msg<< " is unsupported by ";
  msg<< command_type_name<CommandType>();
  msg<< " command publisher";
  throw std::runtime_error(msg.str());
}

}

namespace romea {

//-----------------------------------------------------------------------------
template<class CommandType>
CommandPublisher<CommandType>::CommandPublisher(std::shared_ptr<rclcpp::Node> node,
                                                const std::string & message_type):
  publisher_(make_publisher_(node,message_type))
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
void CommandPublisher<CommandType>::publish(const CommandType & command)
{
  publisher_->publish(command);
}

//-----------------------------------------------------------------------------
template<class CommandType>
std::string CommandPublisher<CommandType>::get_topic_name() const
{
  return publisher_->get_topic_name();
}

//-----------------------------------------------------------------------------
template<>
CommandPublisher<SkidSteeringCommand>::PublisherBasePtr
CommandPublisher<SkidSteeringCommand>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                       const std::string & message_type)
{
  if(message_type == "geometry_msgs/Twist")
  {
    using MsgType = geometry_msgs::msg::Twist;
    return make_publisher_<MsgType>(node,"cmd_vel",1);
  }
  else if( message_type == "romea_mobile_base_msgs/SkidSteeringCommand")
  {
    using MsgType = romea_mobile_base_msgs::msg::SkidSteeringCommand;
    return make_publisher_<MsgType>(node,"cmd_skid_steering",1);
  }
  else
  {
    throw_exception<SkidSteeringCommand>(message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandPublisher<OneAxleSteeringCommand>::PublisherBasePtr
CommandPublisher<OneAxleSteeringCommand>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                          const std::string & message_type)
{
  if(message_type == "geometry_msgs/Twist")
  {
    using MsgType = geometry_msgs::msg::Twist;
    return make_publisher_<MsgType>(node,"cmd_vel",1);
  }
  else if( message_type == "romea_mobile_base_msgs/OneAxleSteeringCommand")
  {
    using MsgType = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
    return make_publisher_<MsgType>(node,"cmd_one_axle_steering",1);
  }
  else
  {
    throw_exception<OneAxleSteeringCommand>(message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandPublisher<OmniSteeringCommand>::PublisherBasePtr
CommandPublisher<OmniSteeringCommand>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                       const std::string & message_type)
{

  if(message_type == "geometry_msgs/Twist")
  {
    using MsgType = geometry_msgs::msg::Twist;
    return make_publisher_<MsgType>(node,"cmd_vel",1);
  }
  else if( message_type == "romea_mobile_base_msgs/OmniSteeringCommand")
  {
    using MsgType = romea_mobile_base_msgs::msg::OmniSteeringCommand;
    return make_publisher_<MsgType>(node,"cmd_omni_steering",1);
  }
  else
  {
    throw_exception<OmniSteeringCommand>(message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandPublisher<TwoAxleSteeringCommand>::PublisherBasePtr
CommandPublisher<TwoAxleSteeringCommand>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                          const std::string & message_type)
{
  if(message_type == "four_wheel_steering_msgs/FourWheelSteering")
  {
    using MsgType = four_wheel_steering_msgs::msg::FourWheelSteering;
    return make_publisher_<MsgType>(node,"cmd_vel",1);
  }
  else if( message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand")
  {
    using MsgType = romea_mobile_base_msgs::msg::TwoAxleSteeringCommand;
    return make_publisher_<MsgType>(node,"cmd_omni_steering",1);
  }
  else
  {
    throw_exception<TwoAxleSteeringCommand>(message_type);
    return {};
  }
}


template class CommandPublisher<SkidSteeringCommand>;
template class CommandPublisher<OmniSteeringCommand>;
template class CommandPublisher<OneAxleSteeringCommand>;
template class CommandPublisher<TwoAxleSteeringCommand>;

}

