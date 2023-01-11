// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// romea
#include <romea_common_utils/listeners/data_listener.hpp>
#include <romea_mobile_base_msgs/msg/skid_steering_command.hpp>

// ros
#include <rclcpp/node.hpp>

// std
#include <string>
#include <memory>

// local
#include "../test/test_helper.h"
#include "romea_mobile_base_utils/control/command_publisher.hpp"

class TestOmniSteeringCommandPublisher : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_command_publisher");
    command.longitudinalSpeed = 1;
    command.lateralSpeed = 2;
    command.angularSpeed = 3;
  }

  void make_publisher(const std::string & message_type)
  {
    publisher = romea::make_command_publisher<romea::OmniSteeringCommand>(node, message_type);
    publisher->activate();
  }

  template<typename MsgType>
  void make_listener(std::string topic_name)
  {
    listener = romea::make_data_listener<romea::OmniSteeringCommand, MsgType>(
      node, topic_name, romea::best_effort(
        1));
  }

  void init(const std::string & message_type)
  {
    make_publisher(message_type);
    if (message_type == "geometry_msgs/Twist") {
      make_listener<geometry_msgs::msg::Twist>("cmd_vel");
    } else if (message_type == "romea_mobile_base_msgs/OmniSteeringCommand") {
      make_listener<romea_mobile_base_msgs::msg::OmniSteeringCommand>("cmd_omni_steering");
    }
  }

  std::shared_ptr<rclcpp::Node> node;
  romea::OmniSteeringCommand command;

  std::shared_ptr<romea::PublisherBase<romea::OmniSteeringCommand>> publisher;
  std::shared_ptr<romea::DataListenerBase<romea::OmniSteeringCommand>> listener;
};


TEST_F(TestOmniSteeringCommandPublisher, checkPublishTwistMessage)
{
  init("geometry_msgs/Twist");

  publisher->publish(command);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  EXPECT_STREQ(publisher->get_topic_name().c_str(), listener->get_topic_name().c_str());
  EXPECT_DOUBLE_EQ(listener->get_data().longitudinalSpeed, command.longitudinalSpeed);
  EXPECT_DOUBLE_EQ(listener->get_data().lateralSpeed, command.lateralSpeed);
  EXPECT_DOUBLE_EQ(listener->get_data().angularSpeed, command.angularSpeed);
}

TEST_F(TestOmniSteeringCommandPublisher, checkPublishOmniSteeringMessage)
{
  init("romea_mobile_base_msgs/OmniSteeringCommand");

  publisher->publish(command);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  EXPECT_STREQ(publisher->get_topic_name().c_str(), listener->get_topic_name().c_str());
  EXPECT_DOUBLE_EQ(listener->get_data().longitudinalSpeed, command.longitudinalSpeed);
  EXPECT_DOUBLE_EQ(listener->get_data().lateralSpeed, command.lateralSpeed);
  EXPECT_DOUBLE_EQ(listener->get_data().angularSpeed, command.angularSpeed);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
