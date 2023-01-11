// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_PUBLISHER_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_PUBLISHER_HPP_

// romea
#include <romea_common_utils/publishers/data_publisher.hpp>
#include <romea_common_utils/qos.hpp>

// std
#include <memory>
#include <string>

// local
#include "romea_mobile_base_utils/conversions/command_conversions.hpp"


namespace romea
{

template<typename CommandType, typename MsgType, typename NodeType>
std::shared_ptr<DataPublisher<CommandType, MsgType, NodeType>>
make_command_publisher(
  std::shared_ptr<NodeType> node,
  const std::string & topic_name)
{
  return romea::make_data_publisher<CommandType, MsgType>(
    node, topic_name, romea::reliable(1), false);
}


template<typename CommandType>
struct CommandPublisher
{
};

template<>
struct CommandPublisher<SkidSteeringCommand>
{
  using PubType = PublisherBase<SkidSteeringCommand>;

  template<typename MsgType, typename NodeType>
  static std::shared_ptr<PubType> instance(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name)
  {
    return make_command_publisher<SkidSteeringCommand, MsgType>(node, topic_name);
  }

  template<typename NodeType>
  static std::shared_ptr<PubType> instance(
    std::shared_ptr<NodeType> node,
    const std::string & message_type)
  {
    if (message_type == "geometry_msgs/Twist") {
      using msg = geometry_msgs::msg::Twist;
      return instance<msg>(node, "cmd_vel");
    } else if (message_type == "romea_mobile_base_msgs/SkidSteeringCommand") {
      using msg = romea_mobile_base_msgs::msg::SkidSteeringCommand;
      return instance<msg>(node, "cmd_skid_steering");
    } else {
      throw std::runtime_error(
              "Output message type " + message_type +
              "is unsupported by skid steering command publisher");
    }
  }
};

template<>
struct CommandPublisher<OmniSteeringCommand>
{
  using PubType = PublisherBase<OmniSteeringCommand>;

  template<typename MsgType, typename NodeType>
  static std::shared_ptr<PubType> instance(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name)
  {
    return make_command_publisher<OmniSteeringCommand, MsgType>(node, topic_name);
  }

  template<typename NodeType>
  static std::shared_ptr<PubType> instance(
    std::shared_ptr<NodeType> node,
    const std::string & message_type)
  {
    if (message_type == "geometry_msgs/Twist") {
      using msg = geometry_msgs::msg::Twist;
      return instance<msg>(node, "cmd_vel");
    } else if (message_type == "romea_mobile_base_msgs/OmniSteeringCommand") {
      using msg = romea_mobile_base_msgs::msg::OmniSteeringCommand;
      return instance<msg>(node, "cmd_omni_steering");
    } else {
      throw std::runtime_error(
              "Output message type " + message_type +
              "is unsupported by omni steering command publisher");
    }
  }
};

template<>
struct CommandPublisher<OneAxleSteeringCommand>
{
  using PubType = PublisherBase<OneAxleSteeringCommand>;

  template<typename MsgType, typename NodeType>
  static std::shared_ptr<PubType> instance(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name)
  {
    return make_command_publisher<OneAxleSteeringCommand, MsgType>(node, topic_name);
  }

  template<typename NodeType>
  static std::shared_ptr<PubType> instance(
    std::shared_ptr<NodeType> node,
    const std::string & message_type)
  {
    if (message_type == "geometry_msgs/Twist") {
      using msg = geometry_msgs::msg::Twist;
      return instance<msg>(node, "cmd_vel");
    } else if (message_type == "ackermann_msgs/AckermannDrive") {
      using msg = ackermann_msgs::msg::AckermannDrive;
      return instance<msg>(node, "cmd_steer");
    } else if (message_type == "romea_mobile_base_msgs/OneAxleSteeringCommand") {
      using msg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
      return instance<msg>(node, "cmd_one_axle_steering");
    } else {
      throw std::runtime_error(
              "Output message type " + message_type +
              "is unsupported by one axle steering command publisher");
    }
  }
};

template<>
struct CommandPublisher<TwoAxleSteeringCommand>
{
  using PubType = PublisherBase<TwoAxleSteeringCommand>;

  template<typename MsgType, typename NodeType>
  static std::shared_ptr<PubType> instance(
    std::shared_ptr<NodeType> node,
    const std::string & topic_name)
  {
    return make_command_publisher<TwoAxleSteeringCommand, MsgType>(node, topic_name);
  }

  template<typename NodeType>
  static std::shared_ptr<PubType> instance(
    std::shared_ptr<NodeType> node,
    const std::string & message_type)
  {
    if (message_type == "four_wheel_steering_msgs/FourWheelSteering") {
      using msg = four_wheel_steering_msgs::msg::FourWheelSteering;
      return instance<msg>(node, "cmd_4ws");
    } else if (message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand") {
      using msg = romea_mobile_base_msgs::msg::TwoAxleSteeringCommand;
      return instance<msg>(node, "cmd_two_axle_steering");
    } else {
      throw std::runtime_error(
              "Output message type " + message_type +
              "is unsupported by two axle steering command publisher");
    }
  }
};


template<typename CommandType, typename NodeType>
std::shared_ptr<PublisherBase<CommandType>>
make_command_publisher(
  std::shared_ptr<NodeType> node,
  const std::string message_type)
{
  if constexpr (std::is_same_v<CommandType, SkidSteeringCommand>)
  {
    return CommandPublisher<SkidSteeringCommand>::instance(node, message_type);
  }

  if constexpr (std::is_same_v<CommandType, OmniSteeringCommand>)
  {
    return CommandPublisher<OmniSteeringCommand>::instance(node, message_type);
  }

  if constexpr (std::is_same_v<CommandType, OneAxleSteeringCommand>)
  {
    return CommandPublisher<OneAxleSteeringCommand>::instance(node, message_type);
  }

  if constexpr (std::is_same_v<CommandType, TwoAxleSteeringCommand>)
  {
    return CommandPublisher<TwoAxleSteeringCommand>::instance(node, message_type);
  }
}

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_PUBLISHER_HPP_
