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
#include <string>
#include <memory>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"

// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_utils/params/mobile_base_parameters2FWS4WD.hpp"


class TestMobileBaseParams2FWS4WD : public ::testing::Test
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
        std::string(TEST_DIR) + "/test_mobile_base_parameters_2FWS4WD.yaml"});
    node = std::make_shared<rclcpp::Node>("test_mobile_base_parameters_2FWS4WD", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestMobileBaseParams2FWS4WD, checkGetInfo)
{
  romea::declare_mobile_base_info_2FWS4WD(node, "base");
  auto base_info = romea::get_mobile_base_info_2FWS4WD(node, "base");

  EXPECT_DOUBLE_EQ(base_info.geometry.axlesDistance, 101);
  EXPECT_DOUBLE_EQ(base_info.geometry.frontAxle.wheelsDistance, 102);
  EXPECT_DOUBLE_EQ(base_info.geometry.frontAxle.wheels.radius, 103);
  EXPECT_DOUBLE_EQ(base_info.geometry.frontAxle.wheels.width, 104);
  EXPECT_DOUBLE_EQ(base_info.geometry.frontAxle.wheels.hubCarrierOffset, 105);
  EXPECT_DOUBLE_EQ(base_info.geometry.rearAxle.wheelsDistance, 106);
  EXPECT_DOUBLE_EQ(base_info.geometry.rearAxle.wheels.radius, 107);
  EXPECT_DOUBLE_EQ(base_info.geometry.rearAxle.wheels.width, 108);
  EXPECT_DOUBLE_EQ(base_info.geometry.rearAxle.wheels.hubCarrierOffset, 109);
  EXPECT_DOUBLE_EQ(base_info.frontWheelsSteeringControl.command.maximalAngle, 200);
  EXPECT_DOUBLE_EQ(base_info.frontWheelsSteeringControl.command.maximalAngularSpeed, 201);
  EXPECT_DOUBLE_EQ(base_info.frontWheelsSteeringControl.sensor.angleStd, 202);
  EXPECT_DOUBLE_EQ(base_info.frontWheelsSteeringControl.sensor.angleRange, 203);
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

// TEST_F(TestMobileBaseParams2FWS4WD, checkGetJointMappings)
//{
//  romea::declare_joint_mappings_2FWS4WD(node,"base.joints");
//  auto base_info =romea::get_joint_mappings_2FWS4WD(node,"base.joints");
//}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
