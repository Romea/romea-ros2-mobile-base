//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_utils/params/mobile_base_parameters2WD.hpp"


class TestMobileBaseParams2WD : public ::testing::Test
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
    no.arguments({"--ros-args","--params-file",std::string(TEST_DIR)+"/test_mobile_base_parameters_2TD.yaml"});
    node = std::make_shared<rclcpp::Node>("test_mobile_base_parameters_2TD", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestMobileBaseParams2WD, checkGetInfo)
{
  romea::MobileBaseInfo2TD base_info;
  romea::declare_mobile_base_info_2TD(node,"base");
  romea::get_mobile_base_info_2TD(node,"base",base_info);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracksDistance,100);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.width,102);
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
