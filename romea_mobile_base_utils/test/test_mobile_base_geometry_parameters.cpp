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

  romea::declare_wheel_info(node,"wheeled_geometry.front_axle.wheels");
  romea::Wheel wheel = romea::get_wheel_info(node,"wheeled_geometry.front_axle.wheels");
  EXPECT_DOUBLE_EQ(wheel.radius,103);
  EXPECT_DOUBLE_EQ(wheel.width,104);
  EXPECT_DOUBLE_EQ(wheel.hubCarrierOffset,105);
}

TEST_F(TestMobileBaseGeometryParams, GetFlatContinousTrackInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_flat_continuous_track_info(node,"tracked_geometry.front_axle.tracks");
  auto track =romea::get_flat_continuous_track_info(node,"tracked_geometry.front_axle.tracks");
  EXPECT_DOUBLE_EQ(track.width,111);
  EXPECT_DOUBLE_EQ(track.sprocket_wheel.radius,112);
  EXPECT_DOUBLE_EQ(track.sprocket_wheel.x,113);
  EXPECT_DOUBLE_EQ(track.idler_wheel.radius,114);
  EXPECT_DOUBLE_EQ(track.idler_wheel.x,115);
}

TEST_F(TestMobileBaseGeometryParams, GetTriangularContinousTrackInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_triangular_continuous_track_info(node,"tracked_geometry.rear_axle.tracks");
  auto track =romea::get_triangular_continuous_track_info(node,"tracked_geometry.rear_axle.tracks");
  EXPECT_DOUBLE_EQ(track.width,117);
  EXPECT_DOUBLE_EQ(track.high_sprocket_wheel.radius,118);
  EXPECT_DOUBLE_EQ(track.high_sprocket_wheel.x,119);
  EXPECT_DOUBLE_EQ(track.front_idler_wheel.radius,120);
  EXPECT_DOUBLE_EQ(track.front_idler_wheel.x,121);
  EXPECT_DOUBLE_EQ(track.rear_idler_wheel.radius,122);
  EXPECT_DOUBLE_EQ(track.rear_idler_wheel.x,123);
}

TEST_F(TestMobileBaseGeometryParams, GetWheeledAxleInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_wheeled_axle_info(node,"wheeled_geometry.front_axle");
  auto axle =romea::get_wheeled_axle_info(node,"wheeled_geometry.front_axle");
  EXPECT_DOUBLE_EQ(axle.wheelsDistance,102);
  EXPECT_DOUBLE_EQ(axle.wheels.radius,103);
  EXPECT_DOUBLE_EQ(axle.wheels.width,104);
  EXPECT_DOUBLE_EQ(axle.wheels.hubCarrierOffset,105);
}

TEST_F(TestMobileBaseGeometryParams, GetFlatContinuousTrackedAxleInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_flat_continuous_tracked_axle_info(node,"tracked_geometry.front_axle");
  auto axle =romea::get_flat_continuous_tracked_axle_info(node,"tracked_geometry.front_axle");
  EXPECT_DOUBLE_EQ(axle.tracksDistance,110);
  EXPECT_DOUBLE_EQ(axle.tracks.width,111);
  EXPECT_DOUBLE_EQ(axle.tracks.sprocket_wheel.radius,112);
  EXPECT_DOUBLE_EQ(axle.tracks.sprocket_wheel.x,113);
  EXPECT_DOUBLE_EQ(axle.tracks.idler_wheel.radius,114);
  EXPECT_DOUBLE_EQ(axle.tracks.idler_wheel.x,115);

}

TEST_F(TestMobileBaseGeometryParams, GetTriangularContinuousTrackedAxleInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_triangular_continuous_tracked_axle_info(node,"tracked_geometry.rear_axle");
  auto axle =romea::get_triangular_continuous_tracked_axle_info(node,"tracked_geometry.rear_axle");
  EXPECT_DOUBLE_EQ(axle.tracksDistance,116);
  EXPECT_DOUBLE_EQ(axle.tracks.width,117);
  EXPECT_DOUBLE_EQ(axle.tracks.high_sprocket_wheel.radius,118);
  EXPECT_DOUBLE_EQ(axle.tracks.high_sprocket_wheel.x,119);
  EXPECT_DOUBLE_EQ(axle.tracks.front_idler_wheel.radius,120);
  EXPECT_DOUBLE_EQ(axle.tracks.front_idler_wheel.x,121);
  EXPECT_DOUBLE_EQ(axle.tracks.rear_idler_wheel.radius,122);
  EXPECT_DOUBLE_EQ(axle.tracks.rear_idler_wheel.x,123);

}

TEST_F(TestMobileBaseGeometryParams, GetTwoWheelAxlesInfo)
{
  loadYaml(std::string(TEST_DIR)+"/test_mobile_base_geometry_parameters.yaml");

  romea::declare_two_wheeled_axles_info(node,"wheeled_geometry");
  auto axles =romea::get_two_wheeled_axles_info(node,"wheeled_geometry");
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


