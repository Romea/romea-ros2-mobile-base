// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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
#include "romea_mobile_base_gazebo/gazebo_interface2TD.hpp"

class TestGazeboInterface2TD : public gazebo::ServerFixture {};


TEST_F(TestGazeboInterface2TD, testSetGet)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("2TD");
  std::string sdf_description = make_sdf_description("2TD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface2TD gazebo_interface(GetModel("robot"), hardware_info[0], "velocity");

  romea::SimulationCommand2TD command = {-1.0, 1.0, -2.0, 2.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(
    command.leftSprocketWheelSpinningSetPoint,
    state.leftSprocketWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.rightSprocketWheelSpinningSetPoint,
    state.rightSprocketWheelSpinningMotion.velocity, 0.1);
  EXPECT_NEAR(
    command.leftIdlerWheelSpinningSetPoint,
    state.leftIdlerWheelSpinningMotion.velocity,
    0.1);
  EXPECT_NEAR(
    command.rightIdlerWheelSpinningSetPoint,
    state.rightIdlerWheelSpinningMotion.velocity,
    0.1);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
