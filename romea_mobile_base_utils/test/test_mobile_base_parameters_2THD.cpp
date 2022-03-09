//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_utils/params/mobile_base_parameters2THD.hpp"


class TestMobileBaseParams2THD : public ::testing::Test
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
    no.arguments({"--ros-args","--params-file",std::string(TEST_DIR)+"/test_mobile_base_parameters_2THD.yaml"});
    node = std::make_shared<rclcpp::Node>("test_mobile_base_parameters_2THD", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestMobileBaseParams2THD, checkGetInfo)
{
  romea::declare_mobile_base_info_2THD(node,"base");
  auto base_info = romea::get_mobile_base_info_2THD(node,"base");
  EXPECT_DOUBLE_EQ(base_info.geometry.tracksDistance,100);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.width,102);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.high_sprocket_wheel.radius,103);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.high_sprocket_wheel.x,104);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.front_idler_wheel.radius,105);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.front_idler_wheel.x,106);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.rear_idler_wheel.radius,107);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.rear_idler_wheel.x,108);
  EXPECT_DOUBLE_EQ(base_info.tracksSpeedControl.command.maximalSpeed,300);
  EXPECT_DOUBLE_EQ(base_info.tracksSpeedControl.command.maximalAcceleration,301);
  EXPECT_DOUBLE_EQ(base_info.tracksSpeedControl.sensor.speedStd,302);
  EXPECT_DOUBLE_EQ(base_info.tracksSpeedControl.sensor.speedRange,303);
  EXPECT_DOUBLE_EQ(base_info.inertia.mass,400);
  EXPECT_DOUBLE_EQ(base_info.inertia.center.x(),401);
  EXPECT_DOUBLE_EQ(base_info.inertia.center.y(),402);
  EXPECT_DOUBLE_EQ(base_info.inertia.center.z(),403);
  EXPECT_DOUBLE_EQ(base_info.inertia.zMoment,404);
  EXPECT_DOUBLE_EQ(base_info.controlPoint.x(),500);
  EXPECT_DOUBLE_EQ(base_info.controlPoint.y(),501);
  EXPECT_DOUBLE_EQ(base_info.controlPoint.z(),502);
}

TEST_F(TestMobileBaseParams2THD, checkGetJointMappings)
{
  romea::declare_joint_mappings_2THD(node,"base.joints");
  auto base_info =romea::get_joint_mappings_2THD(node,"base.joints");
}
