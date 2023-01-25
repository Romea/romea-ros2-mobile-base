// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>
#include <string>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"

// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_utils/params/mobile_base_parameters4WD.hpp"


class TestMobileBaseParams4WD : public ::testing::Test
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
        std::string(TEST_DIR) + "/test_mobile_base_parameters_4WD.yaml"});
    node = std::make_shared<rclcpp::Node>("test_mobile_base_parameters_4WD", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestMobileBaseParams4WD, checkGetInfo)
{
  romea::declare_mobile_base_info_4WD(node, "base");
  auto base_info = romea::get_mobile_base_info_4WD(node, "base");

  EXPECT_DOUBLE_EQ(base_info.geometry.axlesDistance, 101);
  EXPECT_DOUBLE_EQ(base_info.geometry.frontAxle.wheelsDistance, 102);
  EXPECT_DOUBLE_EQ(base_info.geometry.frontAxle.wheels.radius, 103);
  EXPECT_DOUBLE_EQ(base_info.geometry.frontAxle.wheels.width, 104);
  EXPECT_DOUBLE_EQ(base_info.geometry.frontAxle.wheels.hubCarrierOffset, 105);
  EXPECT_DOUBLE_EQ(base_info.geometry.rearAxle.wheelsDistance, 106);
  EXPECT_DOUBLE_EQ(base_info.geometry.rearAxle.wheels.radius, 107);
  EXPECT_DOUBLE_EQ(base_info.geometry.rearAxle.wheels.width, 108);
  EXPECT_DOUBLE_EQ(base_info.geometry.rearAxle.wheels.hubCarrierOffset, 109);
  EXPECT_DOUBLE_EQ(base_info.wheelsSpeedControl.command.maximalSpeed, 300);
  EXPECT_DOUBLE_EQ(base_info.wheelsSpeedControl.command.maximalAcceleration, 301);
  EXPECT_DOUBLE_EQ(base_info.wheelsSpeedControl.sensor.speedStd, 302);
  EXPECT_DOUBLE_EQ(base_info.wheelsSpeedControl.sensor.speedRange, 303);
  EXPECT_DOUBLE_EQ(base_info.inertia.mass, 400);
  EXPECT_DOUBLE_EQ(base_info.inertia.center.x(), 401);
  EXPECT_DOUBLE_EQ(base_info.inertia.center.y(), 402);
  EXPECT_DOUBLE_EQ(base_info.inertia.center.z(), 403);
  EXPECT_DOUBLE_EQ(base_info.inertia.zMoment, 404);
  EXPECT_DOUBLE_EQ(base_info.controlPoint.x(), 500);
  EXPECT_DOUBLE_EQ(base_info.controlPoint.y(), 501);
  EXPECT_DOUBLE_EQ(base_info.controlPoint.z(), 502);
}

// TEST_F(TestMobileBaseParams4WD, checkGetJointMappings)
//{
//  romea::declare_joint_mappings_4WD(node,"base.joints");
//  auto base_info =romea::get_joint_mappings_4WD(node,"base.joints");
//}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
