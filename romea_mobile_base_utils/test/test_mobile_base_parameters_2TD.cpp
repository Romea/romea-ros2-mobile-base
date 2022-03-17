//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_utils/params/mobile_base_parameters2TD.hpp"


class TestMobileBaseParams2TD : public ::testing::Test
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


TEST_F(TestMobileBaseParams2TD, checkGetInfo)
{
  romea::declare_mobile_base_info_2TD(node,"base");
  auto base_info = romea::get_mobile_base_info_2TD(node,"base");
  EXPECT_DOUBLE_EQ(base_info.geometry.tracksDistance,100);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.width,102);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.thickness,102.5);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.sprocketWheel.radius,103);
  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.sprocketWheel.x,104);
//  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.idlerWheel.radius,105);
//  EXPECT_DOUBLE_EQ(base_info.geometry.tracks.idlerWheel.x,106);
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

//TEST_F(TestMobileBaseParams2TD, checkGetJointMappingsForTDVehicle)
//{
//  romea::declare_joint_mappings_2TD(node,"base.joints");
//  auto base_info =romea::get_joint_mappings_2TD(node,"base.joints");
//}

//TEST_F(TestMobileBaseParams2TD, checkGetJointMappingsForTTDVehicle)
//{
//  romea::declare_joint_mappings_2TTD(node,"base.joints");
//  auto base_info =romea::get_joint_mappings_2TTD(node,"base.joints");
//}
