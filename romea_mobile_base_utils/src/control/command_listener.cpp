//romea
#include "romea_mobile_base_utils/control/command_listener.hpp"
#include <romea_common_utils/listeners/data_listener.hpp>

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
  msg<< "Message type ";
  msg<< message_type;
  msg<< " is unsupported by ";
  msg<< command_type_name<CommandType>();
  msg<< " command listner";
  throw std::runtime_error(msg.str());
}

}

namespace romea {

//-----------------------------------------------------------------------------
template<class CommandType>
CommandListener<CommandType>::CommandListener(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & message_type):
  listener_(make_listener_(node,message_type))
{
}

//-----------------------------------------------------------------------------
template<class CommandType>
CommandType CommandListener<CommandType>::get_command()const
{
  return listener_->get_data();
}

//-----------------------------------------------------------------------------
template<class CommandType>
std::string CommandListener<CommandType>::get_topic_name() const
{
  return listener_->get_topic_name();
}

//-----------------------------------------------------------------------------
template<>
CommandListener<SkidSteeringCommand>::ListenerBasePtr
CommandListener<SkidSteeringCommand>::make_listener_(std::shared_ptr<rclcpp::Node> node,
                                                     const std::string & message_type)
{
  if(message_type == "geometry_msgs/Twist")
  {
    using MsgType = geometry_msgs::msg::Twist;
    return make_listener_<MsgType>(node,"cmd_vel",1);
  }
  else if( message_type == "romea_mobile_base_msgs/SkidSteeringCommand")
  {
    using MsgType = romea_mobile_base_msgs::msg::SkidSteeringCommand;
    return make_listener_<MsgType>(node,"cmd_skid_steering",1);
  }
  else
  {
    throw_exception<SkidSteeringCommand>(message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandListener<OneAxleSteeringCommand>::ListenerBasePtr
CommandListener<OneAxleSteeringCommand>::make_listener_(std::shared_ptr<rclcpp::Node> node,
                                                        const std::string & message_type)
{
  if(message_type == "geometry_msgs/Twist")
  {
    using MsgType = geometry_msgs::msg::Twist;
    return make_listener_<MsgType>(node,"cmd_vel",1);
  }
  else if( message_type == "romea_mobile_base_msgs/OneAxleSteeringCommand")
  {
    using MsgType = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
    return make_listener_<MsgType>(node,"cmd_one_axle_steering",1);
  }
  else
  {
    throw_exception<OneAxleSteeringCommand>(message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandListener<OmniSteeringCommand>::ListenerBasePtr
CommandListener<OmniSteeringCommand>::make_listener_(std::shared_ptr<rclcpp::Node> node,
                                                     const std::string & message_type)
{

  if(message_type == "geometry_msgs/Twist")
  {
    return make_listener_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
  }
  else if( message_type == "romea_mobile_base_msgs/OmniSteeringCommand")
  {
    return make_listener_<romea_mobile_base_msgs::msg::OmniSteeringCommand>(node,"cmd_omni_steering",1);
  }
  else
  {
    throw_exception<OmniSteeringCommand>(message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandListener<TwoAxleSteeringCommand>::ListenerBasePtr
CommandListener<TwoAxleSteeringCommand>::make_listener_(std::shared_ptr<rclcpp::Node> node,
                                                        const std::string & message_type)
{
  if(message_type == "four_wheel_steering_msgs/FourWheelSteering")
  {
    return make_listener_<four_wheel_steering_msgs::msg::FourWheelSteering>(node,"cmd_vel",1);
  }
  else if( message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand")
  {
    return make_listener_<romea_mobile_base_msgs::msg::TwoAxleSteeringCommand>(node,"cmd_omni_steering",1);
  }
  else
  {
    throw_exception<TwoAxleSteeringCommand>(message_type);
    return {};
  }
}


template class CommandListener<SkidSteeringCommand>;
template class CommandListener<OmniSteeringCommand>;
template class CommandListener<OneAxleSteeringCommand>;
template class CommandListener<TwoAxleSteeringCommand>;

}

