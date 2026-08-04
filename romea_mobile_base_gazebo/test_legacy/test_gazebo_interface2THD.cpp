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
#include "romea_mobile_base_gazebo/gazebo_interface2THD.hpp"

// test
#include "test_fixture.hpp"
#include "test_helper.h"  // NOLINT
#include "test_utils.hpp"

TEST(TestGazeboInterface2ThD, testSetGet)
{
  std::string urdf = make_urdf_description("2THD");
  std::string world = create_sdf_world_file("2THD");

  TestGazeboInterfaceFixture<romea::ros2::GazeboInterface2THD> fixture(world, urdf);
  fixture.Simulator();

  romea::core::SimulationCommand2THD command = {-1.0, 1.0, -2.0, 2.0, 3.0, -3.0};
  for (size_t i = 0; i < 1000; ++i) {
    fixture.interface->set_command(command);
    fixture.Step();
  }
  auto state = fixture.interface->get_state();

  EXPECT_NEAR(
    command.leftSprocketWheelSpinningSetPoint, state.leftSprocketWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rightSprocketWheelSpinningSetPoint,
    state.rightSprocketWheelSpinningMotion.velocity,
    0.1);  // NOLINT
  EXPECT_NEAR(
    command.frontLeftIdlerWheelSpinningSetPoint,
    state.frontLeftIdlerWheelSpinningMotion.velocity,
    0.1);  // NOLINT
  EXPECT_NEAR(
    command.frontRightIdlerWheelSpinningSetPoint,
    state.frontRightIdlerWheelSpinningMotion.velocity,
    0.1);  // NOLINT
  EXPECT_NEAR(
    command.rearLeftIdlerWheelSpinningSetPoint,
    state.rearLeftIdlerWheelSpinningMotion.velocity,
    0.1);  // NOLINT
  EXPECT_NEAR(
    command.rearRightIdlerWheelSpinningSetPoint,
    state.rearRightIdlerWheelSpinningMotion.velocity,
    0.1);  // NOLINT
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
