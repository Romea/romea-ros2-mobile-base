// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <memory>
#include <string>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/rclcpp.hpp"

// local
#include "../test/test_helper.h"
#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"

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
    no.arguments({"--ros-args", "--params-file", config_filename});
    node = std::make_shared<rclcpp::Node>("test_mobile_base_geometry_paramerters", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestMobileBaseGeometryParams, GetWheeInfo)
{
  loadYaml(std::string(TEST_DIR) + "/test_mobile_base_geometry_parameters.yaml");

  romea::declare_wheel_info(node, "wheeled_geometry.front_axle.wheels");
  romea::Wheel wheel = romea::get_wheel_info(node, "wheeled_geometry.front_axle.wheels");
  EXPECT_DOUBLE_EQ(wheel.radius, 103);
  EXPECT_DOUBLE_EQ(wheel.width, 104);
  EXPECT_DOUBLE_EQ(wheel.hubCarrierOffset, 105);
}

TEST_F(TestMobileBaseGeometryParams, GetContinousTrackInfo)
{
  loadYaml(std::string(TEST_DIR) + "/test_mobile_base_geometry_parameters.yaml");

  romea::declare_continuous_track_info(node, "tracked_geometry.front_axle.tracks");
  auto track = romea::get_continuous_track_info(node, "tracked_geometry.front_axle.tracks");
  EXPECT_DOUBLE_EQ(track.width, 111);
  EXPECT_DOUBLE_EQ(track.thickness, 111.5);
  EXPECT_DOUBLE_EQ(track.sprocketWheel.radius, 112);
  EXPECT_DOUBLE_EQ(track.sprocketWheel.z, 112);
  EXPECT_DOUBLE_EQ(track.sprocketWheel.x, 113);
  EXPECT_EQ(track.idlerWheels.size(), 1u);
  EXPECT_DOUBLE_EQ(track.idlerWheels[0].radius, 114);
  EXPECT_DOUBLE_EQ(track.idlerWheels[0].z, 114);
  EXPECT_DOUBLE_EQ(track.idlerWheels[0].x, 115);
  EXPECT_EQ(track.rollerWheels.size(), 2u);
  EXPECT_DOUBLE_EQ(track.rollerWheels[0].radius, 115.1);
  EXPECT_DOUBLE_EQ(track.rollerWheels[0].x, 115.2);
  EXPECT_DOUBLE_EQ(track.rollerWheels[0].z, 115.4);
  EXPECT_DOUBLE_EQ(track.rollerWheels[1].radius, 115.1);
  EXPECT_DOUBLE_EQ(track.rollerWheels[1].x, 115.3);
  EXPECT_DOUBLE_EQ(track.rollerWheels[1].z, 115.4);
}

TEST_F(TestMobileBaseGeometryParams, GetTriangleContinousTrackInfo)
{
  loadYaml(std::string(TEST_DIR) + "/test_mobile_base_geometry_parameters.yaml");

  romea::declare_continuous_track_info(node, "tracked_geometry.rear_axle.tracks");
  auto track = romea::get_continuous_track_info(node, "tracked_geometry.rear_axle.tracks");
  EXPECT_DOUBLE_EQ(track.width, 117);
  EXPECT_DOUBLE_EQ(track.thickness, 117.5);
  EXPECT_DOUBLE_EQ(track.sprocketWheel.radius, 118);
  EXPECT_DOUBLE_EQ(track.sprocketWheel.x, 119);
  EXPECT_DOUBLE_EQ(track.sprocketWheel.z, 119.5);
  EXPECT_EQ(track.idlerWheels.size(), 2u);
  EXPECT_DOUBLE_EQ(track.idlerWheels[0].radius, 120);
  EXPECT_DOUBLE_EQ(track.idlerWheels[0].z, 120);
  EXPECT_DOUBLE_EQ(track.idlerWheels[0].x, 121);
  EXPECT_DOUBLE_EQ(track.idlerWheels[1].radius, 122);
  EXPECT_DOUBLE_EQ(track.idlerWheels[1].z, 122);
  EXPECT_DOUBLE_EQ(track.idlerWheels[1].x, 123);
  EXPECT_EQ(track.rollerWheels.size(), 2u);
  EXPECT_DOUBLE_EQ(track.rollerWheels[0].radius, 123.1);
  EXPECT_DOUBLE_EQ(track.rollerWheels[0].x, 123.2);
  EXPECT_DOUBLE_EQ(track.rollerWheels[0].z, 123.1);
  EXPECT_DOUBLE_EQ(track.rollerWheels[1].radius, 123.1);
  EXPECT_DOUBLE_EQ(track.rollerWheels[1].x, 123.3);
  EXPECT_DOUBLE_EQ(track.rollerWheels[1].z, 123.1);
}

TEST_F(TestMobileBaseGeometryParams, GetWheeledAxleInfo)
{
  loadYaml(std::string(TEST_DIR) + "/test_mobile_base_geometry_parameters.yaml");

  romea::declare_wheeled_axle_info(node, "wheeled_geometry.front_axle");
  auto axle = romea::get_wheeled_axle_info(node, "wheeled_geometry.front_axle");
  EXPECT_DOUBLE_EQ(axle.wheelsDistance, 102);
  EXPECT_DOUBLE_EQ(axle.wheels.radius, 103);
  EXPECT_DOUBLE_EQ(axle.wheels.width, 104);
  EXPECT_DOUBLE_EQ(axle.wheels.hubCarrierOffset, 105);
}

TEST_F(TestMobileBaseGeometryParams, GetContinuousTrackedAxleInfo)
{
  loadYaml(std::string(TEST_DIR) + "/test_mobile_base_geometry_parameters.yaml");

  romea::declare_continuous_tracked_axle_info(node, "tracked_geometry.front_axle");
  auto axle = romea::get_continuous_tracked_axle_info(node, "tracked_geometry.front_axle");
  EXPECT_DOUBLE_EQ(axle.tracksDistance, 110);
  EXPECT_DOUBLE_EQ(axle.tracks.width, 111);
  EXPECT_DOUBLE_EQ(axle.tracks.thickness, 111.5);
  EXPECT_DOUBLE_EQ(axle.tracks.sprocketWheel.radius, 112);
  EXPECT_DOUBLE_EQ(axle.tracks.sprocketWheel.z, 112);
  EXPECT_DOUBLE_EQ(axle.tracks.sprocketWheel.x, 113);
  EXPECT_EQ(axle.tracks.idlerWheels.size(), 1u);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[0].radius, 114);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[0].z, 114);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[0].x, 115);
  EXPECT_EQ(axle.tracks.rollerWheels.size(), 2u);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[0].radius, 115.1);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[0].x, 115.2);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[0].z, 115.4);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[1].radius, 115.1);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[1].x, 115.3);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[1].z, 115.4);
}

TEST_F(TestMobileBaseGeometryParams, GetTriangleContinuousTrackedAxleInfo)
{
  loadYaml(std::string(TEST_DIR) + "/test_mobile_base_geometry_parameters.yaml");

  romea::declare_continuous_tracked_axle_info(node, "tracked_geometry.rear_axle");
  auto axle = romea::get_continuous_tracked_axle_info(node, "tracked_geometry.rear_axle");
  EXPECT_DOUBLE_EQ(axle.tracksDistance, 116);
  EXPECT_DOUBLE_EQ(axle.tracks.width, 117);
  EXPECT_DOUBLE_EQ(axle.tracks.thickness, 117.5);
  EXPECT_DOUBLE_EQ(axle.tracks.sprocketWheel.radius, 118);
  EXPECT_DOUBLE_EQ(axle.tracks.sprocketWheel.x, 119);
  EXPECT_DOUBLE_EQ(axle.tracks.sprocketWheel.z, 119.5);
  EXPECT_EQ(axle.tracks.idlerWheels.size(), 2u);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[0].radius, 120);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[0].z, 120);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[0].x, 121);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[1].radius, 122);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[1].z, 122);
  EXPECT_DOUBLE_EQ(axle.tracks.idlerWheels[1].x, 123);
  EXPECT_EQ(axle.tracks.rollerWheels.size(), 2u);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[0].radius, 123.1);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[0].x, 123.2);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[0].z, 123.1);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[1].radius, 123.1);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[1].x, 123.3);
  EXPECT_DOUBLE_EQ(axle.tracks.rollerWheels[1].z, 123.1);
}

TEST_F(TestMobileBaseGeometryParams, GetTwoWheelAxlesInfo)
{
  loadYaml(std::string(TEST_DIR) + "/test_mobile_base_geometry_parameters.yaml");

  romea::declare_two_wheeled_axles_info(node, "wheeled_geometry");
  auto axles = romea::get_two_wheeled_axles_info(node, "wheeled_geometry");
  EXPECT_DOUBLE_EQ(axles.axlesDistance, 101);
  EXPECT_DOUBLE_EQ(axles.frontAxle.wheelsDistance, 102);
  EXPECT_DOUBLE_EQ(axles.frontAxle.wheels.radius, 103);
  EXPECT_DOUBLE_EQ(axles.frontAxle.wheels.width, 104);
  EXPECT_DOUBLE_EQ(axles.frontAxle.wheels.hubCarrierOffset, 105);
  EXPECT_DOUBLE_EQ(axles.rearAxle.wheelsDistance, 106);
  EXPECT_DOUBLE_EQ(axles.rearAxle.wheels.radius, 107);
  EXPECT_DOUBLE_EQ(axles.rearAxle.wheels.width, 108);
  EXPECT_DOUBLE_EQ(axles.rearAxle.wheels.hubCarrierOffset, 109);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
