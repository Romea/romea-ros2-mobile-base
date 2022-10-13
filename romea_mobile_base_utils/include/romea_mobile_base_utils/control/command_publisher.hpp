#ifndef _romea_CommandPublisher_hpp_
#define _romea_CommandPublisher_hpp_

//romea
#include "../conversions/command_conversions.hpp"
#include <romea_common_utils/publishers/data_publisher.hpp>
#include <romea_common_utils/qos.hpp>


namespace romea {


template <typename CommandType, typename MsgType,typename NodeType>
std::shared_ptr<DataPublisher<CommandType,MsgType,NodeType>>
make_command_publisher(std::shared_ptr<NodeType> node,
                       const std::string topic_name)
{

  return romea::make_data_publisher<CommandType,MsgType>(
        node,topic_name,romea::reliable(1),false);
}


template <typename CommandType>
struct CommandPublisher
{

};

template <>
struct CommandPublisher <SkidSteeringCommand>
{
  using PubType  = PublisherBase<SkidSteeringCommand>;

  template <typename MsgType,typename NodeType>
  static std::shared_ptr<PubType> instance(std::shared_ptr<NodeType> node,
                                           const std::string & topic_name)
  {
    return make_command_publisher<SkidSteeringCommand,MsgType>(node,topic_name);
  }

  template <typename NodeType>
  static std::shared_ptr<PubType> instance(std::shared_ptr<NodeType> node,
                                           const std::string & message_type)
  {
    if(message_type == "geometry_msgs/Twist")
    {
      return instance<geometry_msgs::msg::Twist>(node,"cmd_vel");
    }
    else if( message_type == "romea_mobile_base_msgs/SkidSteeringCommand")
    {
      return instance<romea_mobile_base_msgs::msg::SkidSteeringCommand>(node,"cmd_skid_steering");
    }
    else
    {
      //      throw_exception<SkidSteeringCommand>("skid_steering",message_type);
      return nullptr;
    }

  }
};

template<>
struct CommandPublisher <OmniSteeringCommand>
{
  using PubType  = PublisherBase<OmniSteeringCommand>;

  template <typename MsgType, typename NodeType>
  static std::shared_ptr<PubType> instance(std::shared_ptr<NodeType> node,
                                           const std::string & topic_name)
  {
    return make_command_publisher<OmniSteeringCommand,MsgType>(node,topic_name);
  }

  template <typename NodeType>
  static std::shared_ptr<PubType> instance(std::shared_ptr<NodeType> node,
                                           const std::string & message_type)
  {
    if(message_type == "geometry_msgs/Twist")
    {
      return instance<geometry_msgs::msg::Twist>(node,"cmd_vel");
    }
    else if( message_type == "romea_mobile_base_msgs/OmniSteeringCommand")
    {
      return instance<romea_mobile_base_msgs::msg::OmniSteeringCommand>(node,"cmd_omni_steering");
    }
    else
    {
      //      throw_exception<OmniSteeringCommand>("omni_steering",message_type);
      return nullptr;
    }
  }
};

template<>
struct CommandPublisher <OneAxleSteeringCommand>
{
  using PubType  = PublisherBase<OneAxleSteeringCommand>;

  template <typename MsgType, typename NodeType>
  static std::shared_ptr<PubType> instance(std::shared_ptr<NodeType> node,
                                           const std::string & topic_name)
  {
    return make_command_publisher<OneAxleSteeringCommand,MsgType>(node,topic_name);
  }

  template <typename NodeType>
  static std::shared_ptr<PubType> instance(std::shared_ptr<NodeType> node,
                                           const std::string & message_type)
  {
    if(message_type == "geometry_msgs/Twist")
    {
      return instance<geometry_msgs::msg::Twist>(node,"cmd_vel");
    }
    else if( message_type == "ackermann_msgs/AckermannDrive")
    {
      return instance<ackermann_msgs::msg::AckermannDrive>(node,"cmd_steer");
    }
    else if( message_type == "romea_mobile_base_msgs/OneAxleSteeringCommand")
    {
      return instance<romea_mobile_base_msgs::msg::OneAxleSteeringCommand>(node,"cmd_one_axle_steering");
    }
    else
    {
      //      throw_exception<OneAxleSteeringCommand>("one_axle_steering",message_type);
      return nullptr;
    }
  }
};

template<>
struct CommandPublisher <TwoAxleSteeringCommand>
{
  using PubType  = PublisherBase<TwoAxleSteeringCommand>;

  template <typename MsgType, typename NodeType>
  static std::shared_ptr<PubType> instance(std::shared_ptr<NodeType> node,
                                           const std::string & topic_name)
  {
    return make_command_publisher<TwoAxleSteeringCommand,MsgType>(node,topic_name);
  }

  template <typename NodeType>
  static std::shared_ptr<PubType> instance(std::shared_ptr<NodeType> node,
                                           const std::string & message_type)
  {
    if(message_type == "four_wheel_steering_msgs/FourWheelSteering")
    {
      return instance<four_wheel_steering_msgs::msg::FourWheelSteering>(node,"cmd_4ws");
    }
    else if( message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand")
    {
      return instance<romea_mobile_base_msgs::msg::TwoAxleSteeringCommand>(node,"cmd_two_axle_steering");
    }
    else
    {
      //      throw_exception<TwoAxleSteeringCommand>("two_axle_steering",message_type);
      return nullptr;
    }
  }
};


template <typename CommandType, typename NodeType>
std::shared_ptr<PublisherBase<CommandType>>
make_command_publisher(std::shared_ptr<NodeType> node,
                       const std::string message_type)
{
  if constexpr(std::is_same_v<CommandType,SkidSteeringCommand>)
  {
    return CommandPublisher<SkidSteeringCommand>::instance(node,message_type);
  }

  if constexpr(std::is_same_v<CommandType,OmniSteeringCommand>)
  {
    return CommandPublisher<OmniSteeringCommand>::instance(node,message_type);
  }

  if constexpr(std::is_same_v<CommandType,OneAxleSteeringCommand>)
  {
    return CommandPublisher<OneAxleSteeringCommand>::instance(node,message_type);
  }

  if constexpr(std::is_same_v<CommandType,TwoAxleSteeringCommand>)
  {
    return CommandPublisher<TwoAxleSteeringCommand>::instance(node,message_type);
  }

}


}
#endif
