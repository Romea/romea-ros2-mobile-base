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
#include "rclcpp/node.hpp"
#include "hardware_interface/component_parser.hpp"

// gazebo
#include "gazebo/test/ServerFixture.hh"


// romea
#include "../test/test_helper.h"
#include "test_utils.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2TTD.hpp"

class TestGazeboInterface2TTD : public gazebo::ServerFixture {};


TEST_F(TestGazeboInterface2TTD, testSetGet)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("2TTD");
  std::string sdf_description = make_sdf_description("2TTD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::ros2::GazeboInterface2TTD gazebo_interface(GetModel("robot"), hardware_info[0],
    "velocity");

  romea::core::SimulationCommand2TTD command = {-1.0, 1.0, -2.0, 2.0, 3.0, -3.0, 4.0, -4.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(
    command.leftSprocketWheelSpinningSetPoint,
    state.leftSprocketWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.rightSprocketWheelSpinningSetPoint,
    state.rightSprocketWheelSpinningMotion.velocity,
    0.1);

  EXPECT_NEAR(
    command.leftIdlerWheelSpinningSetPoint,
    state.leftIdlerWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.rightIdlerWheelSpinningSetPoint,
    state.rightIdlerWheelSpinningMotion.velocity,
    0.1);

  EXPECT_NEAR(
    command.frontLeftRollerWheelSpinningSetPoint,
    state.frontLeftRollerWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.frontRightRollerWheelSpinningSetPoint,
    state.frontRightRollerWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.rearLeftRollerWheelSpinningSetPoint,
    state.rearLeftRollerWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.rearRightRollerWheelSpinningSetPoint,
    state.rearRightRollerWheelSpinningMotion.velocity,
    0.1);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
