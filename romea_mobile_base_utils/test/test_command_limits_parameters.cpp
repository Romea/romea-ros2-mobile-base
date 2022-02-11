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

  romea::declare_command_limits<romea::SkidSteeringCommandLimits>(
        node,"skid_steering_command_limits");

  romea::get_command_limits<romea::SkidSteeringCommandLimits>(
        node,"skid_steering_command_limits",limits);

  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(),-1);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(),2);
  EXPECT_DOUBLE_EQ(limits.angularSpeed.upper(),0.3);
}

TEST_F(TestCommandLimits, getOmniSteeringCommandLimits)
{
  romea::OmniSteeringCommandLimits limits;

  romea::declare_command_limits<romea::OmniSteeringCommandLimits>(
        node,"omni_steering_command_limits");

  romea::get_command_limits<romea::OmniSteeringCommandLimits>(
        node,"omni_steering_command_limits",limits);

  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(),0);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(),1);
  EXPECT_DOUBLE_EQ(limits.lateralSpeed.upper(),1);
  EXPECT_DOUBLE_EQ(limits.angularSpeed.upper(),0.5);
}

TEST_F(TestCommandLimits, getOneAxleSteeringCommandLimits)
{
  romea::OneAxleSteeringCommandLimits limits;

  romea::declare_command_limits<romea::OneAxleSteeringCommandLimits>(
        node,"one_axle_steering_command_limits");

  romea::get_command_limits<romea::OneAxleSteeringCommandLimits>(
        node,"one_axle_steering_command_limits",limits);

  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(),-2);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(),0);
  EXPECT_DOUBLE_EQ(limits.steeringAngle.upper(),0.7);
}

TEST_F(TestCommandLimits, getTwoAxleSteeringCommandLimits)
{
  romea::TwoAxleSteeringCommandLimits limits;

  romea::declare_command_limits<romea::TwoAxleSteeringCommandLimits>(
        node,"two_axle_steering_command_limits");

  romea::get_command_limits<romea::TwoAxleSteeringCommandLimits>(
        node,"two_axle_steering_command_limits",limits);

  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(),-1);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(),2);
  EXPECT_DOUBLE_EQ(limits.frontSteeringAngle.upper(),1.);
  EXPECT_DOUBLE_EQ(limits.rearSteeringAngle.upper(),0.3);

}

//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "ros_param_test");

//  int ret = RUN_ALL_TESTS();
//  ros::shutdown();
//  return ret;
//}
