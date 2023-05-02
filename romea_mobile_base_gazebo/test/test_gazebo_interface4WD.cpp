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
#include "romea_mobile_base_gazebo/gazebo_interface4WD.hpp"

class TestGazeboInterface4WD : public gazebo::ServerFixture {};


TEST_F(TestGazeboInterface4WD, testSetGet)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("4WD");
  std::string sdf_description = make_sdf_description("4WD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface4WD gazebo_interface(GetModel("robot"), hardware_info[0], "velocity");

  romea::SimulationCommand4WD command = {-1.0, 1.0, -2.0, 2.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(
    command.frontLeftWheelSpinningSetPoint,
    state.frontLeftWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.frontRightWheelSpinningSetPoint,
    state.frontRightWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.rearLeftWheelSpinningSetPoint,
    state.rearLeftWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.rearRightWheelSpinningSetPoint,
    state.rearRightWheelSpinningMotion.velocity,
    0.1);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
