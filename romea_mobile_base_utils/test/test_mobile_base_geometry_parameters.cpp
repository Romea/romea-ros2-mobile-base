//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include <romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp>

class TestMobileBaseGeometryParams : public ::testing::Test
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
  }

  void loadYaml(const std::string & config_filename)
  {
    rclcpp::NodeOptions no;
    no.arguments({"--ros-args","--params-file",config_filename});
    node = std::make_shared<rclcpp::Node>("test_mobile_base_geometry_paramerters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestMobileBaseGeometryParams, GetWheeInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_wheel_info(node,"geometry.front_axle.wheels");
  romea::Wheel wheel = romea::get_wheel_info(node,"geometry.front_axle.wheels");
  EXPECT_DOUBLE_EQ(wheel.radius,103);
  EXPECT_DOUBLE_EQ(wheel.width,104);
  EXPECT_DOUBLE_EQ(wheel.hubCarrierOffset,105);
}

TEST_F(TestMobileBaseGeometryParams, GetContinousTrackInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_continuous_track_info(node,"geometry.tracks");
  auto track =romea::get_continuous_track_info(node,"geometry.tracks");
  EXPECT_DOUBLE_EQ(track.width,111);
}

TEST_F(TestMobileBaseGeometryParams, GetWheeledAxleInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_wheeled_axle_info(node,"geometry.front_axle");
  auto axle =romea::get_wheeled_axle_info(node,"geometry.front_axle");
  EXPECT_DOUBLE_EQ(axle.wheelsDistance,102);
  EXPECT_DOUBLE_EQ(axle.wheels.radius,103);
  EXPECT_DOUBLE_EQ(axle.wheels.width,104);
  EXPECT_DOUBLE_EQ(axle.wheels.hubCarrierOffset,105);
}

TEST_F(TestMobileBaseGeometryParams, GetContinuousTrackedAxleInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_continuous_tracked_axle_info(node,"geometry");
  auto axle =romea::get_continuous_tracked_axle_info(node,"geometry");
  EXPECT_DOUBLE_EQ(axle.tracksDistance,110);
  EXPECT_DOUBLE_EQ(axle.tracks.width,111);
}

TEST_F(TestMobileBaseGeometryParams, GetTwoWheelAxlesInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_two_wheeled_axles_info(node,"geometry");
  auto axles =romea::get_two_wheeled_axles_info(node,"geometry");
  EXPECT_DOUBLE_EQ(axles.axlesDistance,101);
  EXPECT_DOUBLE_EQ(axles.frontAxle.wheelsDistance,102);
  EXPECT_DOUBLE_EQ(axles.frontAxle.wheels.radius,103);
  EXPECT_DOUBLE_EQ(axles.frontAxle.wheels.width,104);
  EXPECT_DOUBLE_EQ(axles.frontAxle.wheels.hubCarrierOffset,105);
  EXPECT_DOUBLE_EQ(axles.rearAxle.wheelsDistance,106);
  EXPECT_DOUBLE_EQ(axles.rearAxle.wheels.radius,107);
  EXPECT_DOUBLE_EQ(axles.rearAxle.wheels.width,108);
  EXPECT_DOUBLE_EQ(axles.rearAxle.wheels.hubCarrierOffset,109);
}


