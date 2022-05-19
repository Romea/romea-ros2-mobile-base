//romea
#include "romea_mobile_base_utils/control/command_publisher.hpp"
#include <romea_common_utils/publishers/message_publisher.hpp>

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
void throw_exception(const std::string & output_message_type)
{
  std::stringstream msg;
  msg<< "Output message type ";
  msg<< output_message_type;
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
                                                const std::string & output_message_type):
  publisher_(make_publisher_(node,output_message_type))
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
const std::string & CommandPublisher<CommandType>::get_topic_name() const
{
  return publisher_->get_topic_name();
}

//-----------------------------------------------------------------------------
template<>
CommandPublisher<SkidSteeringCommand>::PublisherBasePtr
CommandPublisher<SkidSteeringCommand>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                       const std::string & output_message_type)
{
  if(output_message_type == "geometry::msg::Twist")
  {
    return make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
  }
  else if( output_message_type == "romea_mobile_base_msgs::msg::SkidSteeringCommand")
  {
    return make_publisher_<romea_mobile_base_msgs::msg::SkidSteeringCommand>(node,"cmd_skid_steering",1);
  }
  else
  {
    throw_exception<SkidSteeringCommand>(output_message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandPublisher<OneAxleSteeringCommand>::PublisherBasePtr
CommandPublisher<OneAxleSteeringCommand>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                          const std::string & output_message_type)
{
  if(output_message_type == "geometry::msg::Twist")
  {
    return make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
  }
  else if( output_message_type == "romea_mobile_base_msgs::msg::OneAxleSteeringCommand")
  {
    return make_publisher_<romea_mobile_base_msgs::msg::OneAxleSteeringCommand>(node,"cmd_one_axle_steering",1);
  }
  else
  {
    throw_exception<OneAxleSteeringCommand>(output_message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandPublisher<OmniSteeringCommand>::PublisherBasePtr
CommandPublisher<OmniSteeringCommand>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                       const std::string & output_message_type)
{

  if(output_message_type == "geometry::msg::Twist")
  {
    return make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
  }
  else if( output_message_type == "romea_mobile_base_msgs::msg::OmniSteeringCommand")
  {
    return make_publisher_<romea_mobile_base_msgs::msg::OmniSteeringCommand>(node,"cmd_omni_steering",1);
  }
  else
  {
    throw_exception<OmniSteeringCommand>(output_message_type);
    return {};
  }
}

//-----------------------------------------------------------------------------
template<>
CommandPublisher<TwoAxleSteeringCommand>::PublisherBasePtr
CommandPublisher<TwoAxleSteeringCommand>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                          const std::string & output_message_type)
{
  if(output_message_type == "four_wheel_steering_msgs::msg::FourWheelSteering")
  {
    return make_publisher_<four_wheel_steering_msgs::msg::FourWheelSteering>(node,"cmd_vel",1);
  }
  else if( output_message_type == "romea_mobile_base_msgs::msg::TwoAxleSteeringCommand")
  {
    return make_publisher_<romea_mobile_base_msgs::msg::TwoAxleSteeringCommand>(node,"cmd_omni_steering",1);
  }
  else
  {
    throw_exception<TwoAxleSteeringCommand>(output_message_type);
    return {};
  }
}


////-----------------------------------------------------------------------------
//template<class CommandType>
//const std::string &CommandPublisher<CommandType>::getTopic() const
//{
//  return publisher_topic_;
//}

////-----------------------------------------------------------------------------
//template<>
//void CommandPublisher<SkidSteeringCommand>::init(std::shared_ptr<rclcpp::Node> node,
//                                                 const std::string & controller_type)
//{
//  if(controller_type == "diff_drive_controller/DiffDriveController")
//  {
//    make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
//  }
//  else
//  {
//    make_publisher_<romea_mobile_base_msgs::msg::SkidSteeringCommand>(node,"cmd_skid_steering",1);
//  }
//}

////-----------------------------------------------------------------------------
//template<>
//void CommandPublisher<OneAxleSteeringCommand>::init(std::shared_ptr<rclcpp::Node> node,
//                                                    const std::string & controller_type)
//{
//  if(controller_type == "ackermann_steering_controller/AckermannSteeringController" ||
//     controller_type == "minimal_ackermann_controller/MinimalAckermannController")
//  {
//    make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
//  }
//  else
//  {
//    make_publisher_<romea_mobile_base_msgs::msg::OneAxleSteeringCommand>(node,"cmd_one_axle_steeering",1);
//  }
//}

////-----------------------------------------------------------------------------
//template<>
//void CommandPublisher<OmniSteeringCommand>::init(std::shared_ptr<rclcpp::Node> node,
//                                                 const std::string & controller_type)
//{
//  if(controller_type == "campero_controller/CamperoController")
//  {
//    make_publisher_<geometry_msgs::msg::Twist>(node,"cmd_vel",1);
//  }
//  else
//  {
//    make_publisher_<romea_mobile_base_msgs::msg::OmniSteeringCommand>(node,"cmd_omni_steeering",1);
//  }
//}


////-----------------------------------------------------------------------------
//template<>
//void CommandPublisher<TwoAxleSteeringCommand>::init(std::shared_ptr<rclcpp::Node> node,
//                                                    const std::string & controller_type)
//{
//  if(controller_type == "four_wheel_steering_controller/FourWheelSteeringController")
//  {
//    make_publisher_<four_wheel_steering_msgs::msg::FourWheelSteering>(node,"cmd_four_wheel_steering",1);
//  }
//  else
//  {
//    make_publisher_<romea_mobile_base_msgs::msg::TwoAxleSteeringCommand>(node,"cmd_two_axle_steering",1);
//  }
//}


template class CommandPublisher<SkidSteeringCommand>;
template class CommandPublisher<OmniSteeringCommand>;
template class CommandPublisher<OneAxleSteeringCommand>;
template class CommandPublisher<TwoAxleSteeringCommand>;

}

