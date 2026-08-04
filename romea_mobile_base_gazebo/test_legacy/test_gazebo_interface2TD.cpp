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
#include "romea_mobile_base_gazebo/gazebo_interface2TD.hpp"

// test
#include "test_fixture.hpp"
#include "test_helper.h"  // NOLINT
#include "test_utils.hpp"

TEST(TestGazeboInterface2TD, testSetGet)
{
  std::string urdf = make_urdf_description("2TD");
  std::string world = create_sdf_world_file("2TD");

  TestGazeboInterfaceFixture<romea::ros2::GazeboInterface2TD> fixture(world, urdf);
  fixture.Simulator();

  romea::core::SimulationCommand2TD command = {-1.0, 1.0, -2.0, 2.0};
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
    command.leftIdlerWheelSpinningSetPoint, state.leftIdlerWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rightIdlerWheelSpinningSetPoint, state.rightIdlerWheelSpinningMotion.velocity, 0.1);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
