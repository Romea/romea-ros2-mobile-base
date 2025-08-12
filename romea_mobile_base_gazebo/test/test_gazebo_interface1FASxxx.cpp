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

// gtest
#include "gtest/gtest.h"

// ros
#include "hardware_interface/component_parser.hpp"

// romea
#include "romea_mobile_base_gazebo/gazebo_interface1FASxxx.hpp"

// test
#include "../test/test_helper.h"
#include "../test/test_fixture.hpp"
#include "../test/test_utils.hpp"


TEST(TestGazeboInterface1FASxxx, testSetGet1FAS2FWD)
{
  std::string urdf = make_urdf_description("1FASxxx", "1FAS2FWD");
  std::string world = create_sdf_world_file("1FAS2FWD");

  TestGazeboInterfaceFixture<romea::ros2::GazeboInterface1FASxxx> fixture(world, urdf);
  fixture.Simulator();

  romea::core::SimulationCommand1FASxxx command = {0.1, 0.2, 0.3, 2.0, -2.0, 3.0, -3.0};
  for (size_t i=0; i < 1000; ++i) {
    fixture.interface->set_command(command);
    fixture.Step();
  }
  auto state = fixture.interface->get_state();

  EXPECT_NEAR(
    command.frontAxleSteeringAngle, state.frontAxleSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSteeringAngle, state.frontLeftWheelSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontRightWheelSteeringAngle, state.frontRightWheelSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSpinningSetPoint, state.frontLeftWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.frontRightWheelSpinningSetPoint, state.frontRightWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rearLeftWheelSpinningSetPoint, state.rearLeftWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rearRightWheelSpinningSetPoint, state.rearRightWheelSpinningMotion.velocity, 0.1);
}

TEST(TestGazeboInterface1FASxxx, testSetGet1FAS2RWD)
{
  std::string urdf = make_urdf_description("1FASxxx", "1FAS2RWD");
  std::string world = create_sdf_world_file("1FAS2RWD");

  TestGazeboInterfaceFixture<romea::ros2::GazeboInterface1FASxxx> fixture(world, urdf);
  fixture.Simulator();

  romea::core::SimulationCommand1FASxxx command = {0.1, 0.2, 0.3, 2.0, -2.0, 3.0, -3.0};
  for (size_t i=0; i < 1000; ++i) {
    fixture.interface->set_command(command);
    fixture.Step();
  }
  auto state = fixture.interface->get_state();

  EXPECT_NEAR(
    command.frontAxleSteeringAngle, state.frontAxleSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSteeringAngle, state.frontLeftWheelSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontRightWheelSteeringAngle, state.frontRightWheelSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSpinningSetPoint, state.frontLeftWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.frontRightWheelSpinningSetPoint, state.frontRightWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rearLeftWheelSpinningSetPoint, state.rearLeftWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rearRightWheelSpinningSetPoint, state.rearRightWheelSpinningMotion.velocity, 0.1);
}

TEST(TestGazeboInterface1FASxxx, testSetGet1FAS4WD)
{
  std::string urdf = make_urdf_description("1FASxxx", "1FAS4WD");
  std::string world = create_sdf_world_file("1FAS4WD");

  TestGazeboInterfaceFixture<romea::ros2::GazeboInterface1FASxxx> fixture(world, urdf);
  fixture.Simulator();

  romea::core::SimulationCommand1FASxxx command = {0.1, 0.2, 0.3, 2.0, -2.0, 3.0, -3.0};
  for (size_t i=0; i < 1000; ++i) {
    fixture.interface->set_command(command);
    fixture.Step();
  }
  auto state = fixture.interface->get_state();

  EXPECT_NEAR(
    command.frontAxleSteeringAngle, state.frontAxleSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSteeringAngle, state.frontLeftWheelSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontRightWheelSteeringAngle, state.frontRightWheelSteeringAngle, 0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSpinningSetPoint, state.frontLeftWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.frontRightWheelSpinningSetPoint, state.frontRightWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rearLeftWheelSpinningSetPoint, state.rearLeftWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rearRightWheelSpinningSetPoint, state.rearRightWheelSpinningMotion.velocity, 0.1);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
