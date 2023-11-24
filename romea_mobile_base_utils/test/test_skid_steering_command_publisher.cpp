// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <memory>
#include <string>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"

// romea
#include "romea_mobile_base_msgs/msg/skid_steering_command.hpp"
#include "romea_mobile_base_utils/control/command_publisher.hpp"
#include "romea_common_utils/listeners/data_listener.hpp"


// local
#include "../test/test_helper.h"


class TestSkidSteeringCommandPublisher : public ::testing::Test
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
    command.angularSpeed = 2;
  }

  void make_publisher(const std::string & message_type)
  {
    publisher = romea::ros2::make_command_publisher<romea::core::SkidSteeringCommand>(
      node, message_type);
  }

  template<typename MsgType>
  void make_listener(std::string topic_name)
  {
    listener = romea::ros2::make_data_listener<romea::core::SkidSteeringCommand, MsgType>(
      node, topic_name, romea::ros2::best_effort(1));
  }

  void init(const std::string & message_type)
  {
    make_publisher(message_type);

    if (message_type == "geometry_msgs/Twist") {
      make_listener<geometry_msgs::msg::Twist>("cmd_vel");
    } else if (message_type == "romea_mobile_base_msgs/SkidSteeringCommand") {
      make_listener<romea_mobile_base_msgs::msg::SkidSteeringCommand>("cmd_skid_steering");
    }

    publisher->activate();
  }

  std::shared_ptr<rclcpp::Node> node;
  romea::core::SkidSteeringCommand command;

  std::shared_ptr<romea::ros2::PublisherBase<romea::core::SkidSteeringCommand>> publisher;
  std::shared_ptr<romea::ros2::DataListenerBase<romea::core::SkidSteeringCommand>> listener;
};


TEST_F(TestSkidSteeringCommandPublisher, checkPublishTwistMessage)
{
  init("geometry_msgs/Twist");

  publisher->publish(command);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  EXPECT_STREQ(publisher->get_topic_name().c_str(), listener->get_topic_name().c_str());
  EXPECT_DOUBLE_EQ(listener->get_data().longitudinalSpeed, command.longitudinalSpeed);
  EXPECT_DOUBLE_EQ(listener->get_data().angularSpeed, command.angularSpeed);
}

TEST_F(TestSkidSteeringCommandPublisher, checkPublishSkidSteeringMessage)
{
  init("romea_mobile_base_msgs/SkidSteeringCommand");

  publisher->publish(command);
  rclcpp::spin_some(node);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  EXPECT_STREQ(publisher->get_topic_name().c_str(), listener->get_topic_name().c_str());
  EXPECT_DOUBLE_EQ(listener->get_data().longitudinalSpeed, command.longitudinalSpeed);
  EXPECT_DOUBLE_EQ(listener->get_data().angularSpeed, command.angularSpeed);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
