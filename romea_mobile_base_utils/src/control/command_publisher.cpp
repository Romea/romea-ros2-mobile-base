//romea
#include "romea_mobile_base_utils/control/command_publisher.hpp"
#include <romea_common_utils/qos.hpp>


//std
#include <sstream>

namespace
{

//template<class CommandType>
//std::string command_type_name()
//{
//  if constexpr(std::is_same_v<CommandType,romea::SkidSteeringCommand>)
//  {
//    return "skid steering";
//  }
//  else if constexpr(std::is_same_v<CommandType,romea::OmniSteeringCommand>)
//  {
//    return "omni steering";
//  }
//  else if constexpr(std::is_same_v<CommandType,romea::OneAxleSteeringCommand>)
//  {
//    return "one axle steering";
//  }
//  else if constexpr(std::is_same_v<CommandType,romea::TwoAxleSteeringCommand>)
//  {
//    return "two axle steering";
//  }
//}

//template<class CommandType>
//void throw_exception(const std::string & message_type)
//{
//  std::stringstream msg;
//  msg<< "Output message type ";
//  msg<< message_type;
//  msg<< " is unsupported by ";
//  msg<< command_type_name<CommandType>();
//  msg<< " command publisher";
//  throw std::runtime_error(msg.str());
//}

//template<class CommandType , class MsgType>
//std::unique_ptr<romea::DataSerializationBase<CommandType>> make_serialization()
//{
//  return std::make_unique<romea::DataSerialization<CommandType,MsgType>>();
//}


template<class CommandType>
void throw_exception(const std::string & command_type,
                     const std::string & message_type)
{
  std::stringstream msg;
  msg<< "Output message type ";
  msg<< message_type;
  msg<< " is unsupported by ";
  msg<< command_type;
  msg<< " command publisher";
  throw std::runtime_error(msg.str());
}

//template <typename CommandType,typename MsgType, typename NodeType>
//std::shared_ptr<romea::PublisherBase<CommandType>>
//make_command_publisher_impl(std::shared_ptr<NodeType> node,
//                            const std::string &topic_name)
//{
//  return romea::make_data_publisher<CommandType,MsgType>(
//        node,topic_name,romea::reliable(1),false);
//}

}

namespace romea {

////-----------------------------------------------------------------------------
//template<class CommandType>
//CommandPublisher<CommandType>::CommandPublisher(std::shared_ptr<rclcpp::Node> node,
//                                                const std::string & message_type)
//{
//  configure_(node,message_type);
//}

////-----------------------------------------------------------------------------
//template<class CommandType>
//void CommandPublisher<CommandType>::publish(const CommandType & command)
//{
//  rclcpp::SerializedMessage serialized_command_message;
//  serialization_->serialize(command,serialized_command_message);
//  publisher_->publish(serialized_command_message);
//}

////-----------------------------------------------------------------------------
//template<class CommandType>
//std::string CommandPublisher<CommandType>::get_topic_name() const
//{
//  return publisher_->get_topic_name();
//}

////-----------------------------------------------------------------------------
//template<class CommandType>
//std::shared_ptr<rclcpp::GenericPublisher>
//CommandPublisher<CommandType>::make_publisher_(std::shared_ptr<rclcpp::Node> node,
//                                               const std::string & topic_name,
//                                               const std::string & message_type)
//{
//  return node->create_generic_publisher(topic_name,message_type,reliable(1));
//}

//////-----------------------------------------------------------------------------
//template<>
//void CommandPublisher<SkidSteeringCommand>::configure_(std::shared_ptr<rclcpp::Node> node,
//                                                       const std::string & message_type)
//{
//  if(message_type == "geometry_msgs/Twist")
//  {
//    using MsgType = geometry_msgs::msg::Twist;
//    publisher_ = make_publisher_(node,"cmd_vel",message_type);
//    serialization_ = make_serialization<SkidSteeringCommand,MsgType>();
//  }
//  else if( message_type == "romea_mobile_base_msgs/SkidSteeringCommand")
//  {
//    using MsgType = romea_mobile_base_msgs::msg::SkidSteeringCommand;
//    publisher_ = make_publisher_(node,"cmd_skid_steering",message_type);
//    serialization_ = make_serialization<SkidSteeringCommand,MsgType>();
//  }
//  else
//  {
//    throw_exception<SkidSteeringCommand>(message_type);
//  }
//}

////-----------------------------------------------------------------------------
//template<>
//void CommandPublisher<OneAxleSteeringCommand>::configure_(std::shared_ptr<rclcpp::Node> node,
//                                                          const std::string & message_type)
//{
//  if(message_type == "geometry_msgs/Twist")
//  {
//    using MsgType = geometry_msgs::msg::Twist;
//    publisher_ = make_publisher_(node,"cmd_vel",message_type);
//    serialization_ = make_serialization<OneAxleSteeringCommand,MsgType>();
//  }
//  else if( message_type == "ackermann_msgs/AckermannDrive")
//  {
//    using MsgType = ackermann_msgs::msg::AckermannDrive;
//    publisher_ = make_publisher_(node,"cmd_steer",message_type);
//    serialization_ = make_serialization<OneAxleSteeringCommand,MsgType>();
//  }
//  else if( message_type == "romea_mobile_base_msgs/OneAxleSteeringCommand")
//  {
//    using MsgType = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
//    publisher_ = make_publisher_(node,"cmd_one_axle_steering",message_type);
//    serialization_ = make_serialization<OneAxleSteeringCommand,MsgType>();
//  }
//  else
//  {
//    throw_exception<OneAxleSteeringCommand>(message_type);
//  }
//}

////-----------------------------------------------------------------------------
//template<>
//void CommandPublisher<OmniSteeringCommand>::configure_(std::shared_ptr<rclcpp::Node> node,
//                                                       const std::string & message_type)
//{
//  if(message_type == "geometry_msgs/Twist")
//  {
//    using MsgType = geometry_msgs::msg::Twist;
//    publisher_ = make_publisher_(node,"cmd_vel",message_type);
//    serialization_ = make_serialization<OmniSteeringCommand,MsgType>();
//  }
//  else if( message_type == "romea_mobile_base_msgs/OmniSteeringCommand")
//  {
//    using MsgType = romea_mobile_base_msgs::msg::OmniSteeringCommand;
//    publisher_ = make_publisher_(node,"cmd_omni_steering",message_type);
//    serialization_ = make_serialization<OmniSteeringCommand,MsgType>();
//  }
//  else
//  {
//    throw_exception<OmniSteeringCommand>(message_type);
//  }
//}

////-----------------------------------------------------------------------------
//template<>
//void CommandPublisher<TwoAxleSteeringCommand>::configure_(std::shared_ptr<rclcpp::Node> node,
//                                                          const std::string & message_type)
//{
//  if(message_type == "four_wheel_steering_msgs/FourWheelSteering")
//  {
//    using MsgType = four_wheel_steering_msgs::msg::FourWheelSteering;
//    publisher_ = make_publisher_(node,"cmd_4ws",message_type);
//    serialization_ = make_serialization<TwoAxleSteeringCommand,MsgType>();
//  }
//  else if( message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand")
//  {
//    using MsgType = romea_mobile_base_msgs::msg::TwoAxleSteeringCommand;
//    publisher_ = make_publisher_(node,"cmd_two_axle_steering",message_type);
//    serialization_ = make_serialization<TwoAxleSteeringCommand,MsgType>();
//  }
//  else
//  {
//    throw_exception<TwoAxleSteeringCommand>(message_type);
//  }
//}

//template class CommandPublisher<SkidSteeringCommand>;
//template class CommandPublisher<OmniSteeringCommand>;
//template class CommandPublisher<OneAxleSteeringCommand>;
//template class CommandPublisher<TwoAxleSteeringCommand>;


////-----------------------------------------------------------------------------
//template<typename NodeType>
//std::shared_ptr<PublisherBase<SkidSteeringCommand>>
//CommandPublisher<SkidSteeringCommand,NodeType>::instance(std::shared_ptr<NodeType> node,
//                                                   const std::string &message_type)
//{
//  toto<1>();
//  if(message_type == "geometry_msgs/Twist")
//  {
//    using MsgType = geometry_msgs::msg::Twist;
//    return make_command_publisher_impl<SkidSteeringCommand,MsgType>(node,"cmd_vel");
//  }
//  else if( message_type == "romea_mobile_base_msgs/SkidSteeringCommand")
//  {
//    using MsgType = romea_mobile_base_msgs::msg::SkidSteeringCommand;
//    return make_command_publisher_impl<SkidSteeringCommand,MsgType>(node,"cmd_skid_steering");
//  }
//  else
//  {
//    throw_exception<SkidSteeringCommand>("skid_steering",message_type);
//    return nullptr;
//  }
//}

////-----------------------------------------------------------------------------
//template <typename NodeType>
//std::shared_ptr<PublisherBase<OmniSteeringCommand>>
//CommandPublisher<NodeType>::omni_steering_instance(std::shared_ptr<NodeType> node,
//                                                   const std::string & message_type)
//{
//  if(message_type == "geometry_msgs/Twist")
//  {
//    using MsgType = geometry_msgs::msg::Twist;
//    return make_command_publisher_impl<OmniSteeringCommand,MsgType>(node,"cmd_vel");
//  }
//  else if( message_type == "romea_mobile_base_msgs/OmniSteeringCommand")
//  {
//    using MsgType = romea_mobile_base_msgs::msg::OmniSteeringCommand;
//    return make_command_publisher_impl<OmniSteeringCommand,MsgType>(node,"cmd_omni_steering");
//  }
//  else
//  {
//    throw_exception<OmniSteeringCommand>("omni_steering",message_type);
//    return nullptr;
//  }
//}

////-----------------------------------------------------------------------------
//template <typename NodeType>
//std::shared_ptr<PublisherBase<OneAxleSteeringCommand>>
//CommandPublisher<NodeType>::one_axle_steering_instance(std::shared_ptr<NodeType> node,
//                                                       const std::string & message_type)
//{

//  if(message_type == "geometry_msgs/Twist")
//  {
//    using MsgType = geometry_msgs::msg::Twist;
//    return make_command_publisher_impl<OneAxleSteeringCommand,MsgType>(node,"cmd_vel");
//  }
//  else if( message_type == "ackermann_msgs/AckermannDrive")
//  {
//    using MsgType = ackermann_msgs::msg::AckermannDrive;
//    return make_command_publisher_impl<OneAxleSteeringCommand,MsgType>(node,"cmd_steer");
//  }
//  else if( message_type == "romea_mobile_base_msgs/OneAxleSteeringCommand")
//  {
//    using MsgType = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
//    return make_command_publisher_impl<OneAxleSteeringCommand,MsgType>(node,"cmd_one_axle_steering");
//  }
//  else
//  {
//    throw_exception<OneAxleSteeringCommand>("one_axle_steering",message_type);
//    return nullptr;
//  }
//}
////-----------------------------------------------------------------------------
//template <typename NodeType>
//std::shared_ptr<PublisherBase<TwoAxleSteeringCommand>>
//CommandPublisher<NodeType>::two_axle_steering_instance(std::shared_ptr<NodeType> node,
//                                                       const std::string & message_type)
//{
//  if(message_type == "four_wheel_steering_msgs/FourWheelSteering")
//  {
//    using MsgType = four_wheel_steering_msgs::msg::FourWheelSteering;
//    return make_command_publisher_impl<TwoAxleSteeringCommand,MsgType>(node,"cmd_4ws");
//  }
//  else if( message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand")
//  {
//    using MsgType = romea_mobile_base_msgs::msg::TwoAxleSteeringCommand;
//    return make_command_publisher_impl<TwoAxleSteeringCommand,MsgType>(node,"cmd_two_axle_steering");
//  }
//  else
//  {
//    throw_exception<TwoAxleSteeringCommand>("two_axle_steering",message_type);
//    return nullptr;
//  }
//}


//template struct CommandPublisher<rclcpp::Node>;
//template struct CommandPublisher<rclcpp_lifecycle::LifecycleNode>;
}

