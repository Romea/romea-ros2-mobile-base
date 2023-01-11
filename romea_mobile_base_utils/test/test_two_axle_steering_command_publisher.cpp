// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// ros
#include <rclcpp/node.hpp>

// romea
#include <romea_common_utils/listeners/data_listener.hpp>
#include <romea_mobile_base_msgs/msg/skid_steering_command.hpp>

// std
#include <memory>
#include <string>

// local
#include "../test/test_helper.h"
#include "romea_mobile_base_utils/control/command_publisher.hpp"

class TestTwoAxleSteeringCommandPublisher : public ::testing::Test
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
    command.frontSteeringAngle = 2;
    command.rearSteeringAngle = 3;
  }

  void make_publisher(const std::string & message_type)
  {
    publisher = romea::make_command_publisher<romea::TwoAxleSteeringCommand>(node, message_type);
    publisher->activate();
  }

  template<typename MsgType>
  void make_listener(std::string topic_name)
  {
    listener = romea::make_data_listener<romea::TwoAxleSteeringCommand, MsgType>(
      node, topic_name, romea::best_effort(
        1));
  }

  void init(const std::string & message_type)
  {
    make_publisher(message_type);

    if (message_type == "four_wheel_steering_msgs/FourWheelSteering") {
      make_listener<four_wheel_steering_msgs::msg::FourWheelSteering>("cmd_4ws");
    } else if (message_type == "romea_mobile_base_msgs/TwoAxleSteeringCommand") {
      make_listener<romea_mobile_base_msgs::msg::TwoAxleSteeringCommand>("cmd_two_axle_steering");
    }
  }

  std::shared_ptr<rclcpp::Node> node;
  romea::TwoAxleSteeringCommand command;

  std::shared_ptr<romea::PublisherBase<romea::TwoAxleSteeringCommand>> publisher;
  std::shared_ptr<romea::DataListenerBase<romea::TwoAxleSteeringCommand>> listener;
};


TEST_F(TestTwoAxleSteeringCommandPublisher, checkPublishFourWheelSteeringMessage)
{
  init("four_wheel_steering_msgs/FourWheelSteering");

  publisher->publish(command);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  EXPECT_STREQ(publisher->get_topic_name().c_str(), listener->get_topic_name().c_str());
  EXPECT_DOUBLE_EQ(listener->get_data().longitudinalSpeed, command.longitudinalSpeed);
  EXPECT_DOUBLE_EQ(listener->get_data().frontSteeringAngle, command.frontSteeringAngle);
  EXPECT_DOUBLE_EQ(listener->get_data().rearSteeringAngle, command.rearSteeringAngle);
}

TEST_F(TestTwoAxleSteeringCommandPublisher, checkPublishTwoAxleSteeringMessage)
{
  init("romea_mobile_base_msgs/TwoAxleSteeringCommand");

  publisher->publish(command);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  EXPECT_STREQ(publisher->get_topic_name().c_str(), listener->get_topic_name().c_str());
  EXPECT_DOUBLE_EQ(listener->get_data().longitudinalSpeed, command.longitudinalSpeed);
  EXPECT_DOUBLE_EQ(listener->get_data().frontSteeringAngle, command.frontSteeringAngle);
  EXPECT_DOUBLE_EQ(listener->get_data().rearSteeringAngle, command.rearSteeringAngle);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
