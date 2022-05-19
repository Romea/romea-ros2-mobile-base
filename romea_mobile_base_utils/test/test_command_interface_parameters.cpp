//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_utils/params/command_interface_parameters.hpp"

class TestCommandInterfaceParams : public ::testing::Test
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
    rclcpp::NodeOptions no;
    no.arguments({"--ros-args","--params-file",std::string(TEST_DIR)+"/test_command_interface_parameters.yaml"});
    node = std::make_shared<rclcpp::Node>("test_command_interface_paramerters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestCommandInterfaceParams, getParameterWithPriority)
{
  romea::declare_command_interface_configuration(node,"with_priority");

  auto config = romea::get_command_interface_configuration(node,"with_priority");

  EXPECT_STREQ(config.output_message_type.c_str(),"foo");
  EXPECT_EQ(config.priority,127);
  EXPECT_DOUBLE_EQ(config.rate,10.);
}


TEST_F(TestCommandInterfaceParams, getParameterWithoutPriority)
{
  romea::declare_command_interface_configuration(node,"without_priority");

  auto config = romea::get_command_interface_configuration(node,"without_priority");

  EXPECT_STREQ(config.output_message_type.c_str(),"bar");
  EXPECT_EQ(config.priority,0);
  EXPECT_DOUBLE_EQ(config.rate,50.);
}

//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "ros_param_test");

//  int ret = RUN_ALL_TESTS();
//  ros::shutdown();
//  return ret;
//}
