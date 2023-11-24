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
#include "romea_mobile_base_utils/kinematic_factory.hpp"

class TestKinematicParams : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_kinematic_params", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestKinematicParams, load4WDtoSkidSteeringKinematicParams)
{
  loadYaml(std::string(TEST_DIR) + std::string("/data/4WD_params.yaml"));

  romea::NodeParameters node_parameters(node);
  romea::core::SkidSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters, kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelTrack, 0.515);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelSpeed, 3);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration, 1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelSpeedVariance, 0.1 * 0.1);
}


TEST_F(TestKinematicParams, load4WS4WDtoFourWheelSteeringKinematicParams)
{
  loadYaml(std::string(TEST_DIR) + std::string("/data/4WS4WD_params.yaml"));

  romea::NodeParameters node_parameters(node);
  romea::core::FourWheelSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters, kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelBase, 1.2);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelBase, 0.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelTrack, 0.9);
  EXPECT_DOUBLE_EQ(kinematic_parameters.hubCarrierOffset, 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelSpeed, 1.99);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration, 1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAngle, 0.26180);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAngularSpeed, 0.349065);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelSpeedVariance, 0.1 * 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelAngleVariance, 0.017453 * 0.017453);
}


TEST_F(TestKinematicParams, load4WS4WDtoTwoAxleKinematicParams) {
  loadYaml(std::string(TEST_DIR) + std::string("/data/4WS4WD_params.yaml"));

  romea::NodeParameters node_parameters(node);
  romea::core::TwoAxleSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters, kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelBase, 1.2);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelBase, 0.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelTrack, 0.9);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelTrack, 0.9);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontHubCarrierOffset, 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearHubCarrierOffset, 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalWheelSpeed, 1.99);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalWheelSpeed, 1.99);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration, 1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalSteeringAngle, 0.26180);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearMaximalSteeringAngle, 0.26180);
  //  EXPECT_DOUBLE_EQ(parameters.maximalSteeringAngularSpeed,0.017453*0.017453);
}

TEST_F(TestKinematicParams, load2WS4WDtoTwoWheelSteeringKinematicParams) {
  loadYaml(std::string(TEST_DIR) + std::string("/data/2FWS4WD_params.yaml"));

  romea::NodeParameters node_parameters(node);
  romea::core::TwoWheelSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters, kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelBase, 2);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelBase, 0);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelTrack, 1.5);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelTrack, 1.5);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontHubCarrierOffset, 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearHubCarrierOffset, 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalWheelSpeed, 2.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearMaximalWheelSpeed, 2.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration, 1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAngle, 0.93);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAngularSpeed, 0.349065);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelSpeedVariance, 0.1 * 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelAngleVariance, 0.017453 * 0.017453);
}

TEST_F(TestKinematicParams, load2WS4WDtoOneSteeringKinematicParams) {
  loadYaml(std::string(TEST_DIR) + std::string("data/2FWS4WD_params.yaml"));

  romea::NodeParameters node_parameters(node);
  romea::core::OneAxleSteeringKinematic::Parameters kinematic_parameters;
  load_kinematic_params(node_parameters, kinematic_parameters);

  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelBase, 2);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelBase, 0);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontWheelTrack, 1.5);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearWheelTrack, 1.5);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontHubCarrierOffset, 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearHubCarrierOffset, 0.1);
  EXPECT_DOUBLE_EQ(kinematic_parameters.frontMaximalWheelSpeed, 2.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.rearMaximalWheelSpeed, 2.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalWheelAcceleration, 1.);
  EXPECT_DOUBLE_EQ(kinematic_parameters.maximalSteeringAngle, 0.72850784045657924);
  //  EXPECT_DOUBLE_EQ(parameters.maximalSteeringAngularSpeed,0.349065);
  EXPECT_DOUBLE_EQ(kinematic_parameters.wheelSpeedVariance, 0.1 * 0.1);
  //  EXPECT_DOUBLE_EQ(parameters.steeringAngleVariance,0.017453*0.017453);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
