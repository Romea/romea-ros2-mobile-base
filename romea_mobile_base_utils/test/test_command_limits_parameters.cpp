// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// ros
#include <rclcpp/node.hpp>

// std
#include <memory>
#include <string>

// romea
#include "../test/test_helper.h"
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
    no.arguments(
      {"--ros-args", "--params-file",
        std::string(TEST_DIR) + std::string("/test_command_limits_parameters.yaml")});
    node = std::make_shared<rclcpp::Node>("test_command_limits_paramerters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestCommandLimits, getSkidSteeringCommandLimits)
{
  romea::declare_command_limits<romea::SkidSteeringCommandLimits>(
    node, "skid_steering_command_limits");

  auto limits = romea::get_command_limits<romea::SkidSteeringCommandLimits>(
    node, "skid_steering_command_limits");

  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(), -1);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(), 2);
  EXPECT_DOUBLE_EQ(limits.angularSpeed.upper(), 0.3);
}

TEST_F(TestCommandLimits, getOmniSteeringCommandLimits)
{
  romea::declare_command_limits<romea::OmniSteeringCommandLimits>(
    node, "omni_steering_command_limits");

  auto limits = romea::get_command_limits<romea::OmniSteeringCommandLimits>(
    node, "omni_steering_command_limits");

  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(), 0);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(), 1);
  EXPECT_DOUBLE_EQ(limits.lateralSpeed.upper(), 1);
  EXPECT_DOUBLE_EQ(limits.angularSpeed.upper(), 0.5);
}

TEST_F(TestCommandLimits, getOneAxleSteeringCommandLimits)
{
  romea::declare_command_limits<romea::OneAxleSteeringCommandLimits>(
    node, "one_axle_steering_command_limits");

  auto limits = romea::get_command_limits<romea::OneAxleSteeringCommandLimits>(
    node, "one_axle_steering_command_limits");

  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(), -2);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(), 0);
  EXPECT_DOUBLE_EQ(limits.steeringAngle.upper(), 0.7);
}

TEST_F(TestCommandLimits, getTwoAxleSteeringCommandLimits)
{
  romea::declare_command_limits<romea::TwoAxleSteeringCommandLimits>(
    node, "two_axle_steering_command_limits");

  auto limits = romea::get_command_limits<romea::TwoAxleSteeringCommandLimits>(
    node, "two_axle_steering_command_limits");

  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.lower(), -1);
  EXPECT_DOUBLE_EQ(limits.longitudinalSpeed.upper(), 2);
  EXPECT_DOUBLE_EQ(limits.frontSteeringAngle.upper(), 1.);
  EXPECT_DOUBLE_EQ(limits.rearSteeringAngle.upper(), 0.3);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
