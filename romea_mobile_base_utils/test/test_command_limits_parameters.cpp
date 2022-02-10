//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_utils/params/command_limits_parameters.hpp"

class TestCommandLimits : public ::testing::Test
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
      no.arguments({"--ros-args","--params-file",std::string(TEST_DIR)+"/test_command_limits_parameters.yaml"});
      node = std::make_shared<rclcpp::Node>("test_command_limits_paramerters", no);
    }

    std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestCommandLimits, getSkidSteeringCommandLimits)
{
  romea::SkidSteeringCommandLimits limits;
  romea::declare_skid_steering_command_limits(node,"skid_steering_command_limits");
  romea::get_skid_steering_command_limits(node,"skid_steering_command_limits",limits);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(),-1);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(),2);
  EXPECT_DOUBLE_EQ(limits.angularSpeed.upper(),0.3);
}

TEST_F(TestCommandLimits, getOmniSteeringCommandLimits)
{
  romea::OmniSteeringCommandLimits limits;
  romea::declare_omni_steering_command_limits(node,"omni_steering_command_limits");
  romea::get_omni_steering_command_limits(node,"omni_steering_command_limits",limits);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(),0);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(),1);
  EXPECT_DOUBLE_EQ(limits.lateralSpeed.upper(),1);
  EXPECT_DOUBLE_EQ(limits.angularSpeed.upper(),0.5);
}


//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "ros_param_test");

//  int ret = RUN_ALL_TESTS();
//  ros::shutdown();
//  return ret;
//}
