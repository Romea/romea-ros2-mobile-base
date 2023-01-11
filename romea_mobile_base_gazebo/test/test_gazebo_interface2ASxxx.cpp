// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// ros
#include <rclcpp/node.hpp>
#include <hardware_interface/component_parser.hpp>

// gazebo
#include <gazebo/test/ServerFixture.hh>

// std
#include <string>

// local
#include "../test/test_helper.h"
#include "test_utils.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2ASxxx.hpp"

class TestGazeboInterface2ASxxx : public gazebo::ServerFixture {};


TEST_F(TestGazeboInterface2ASxxx, testSetGet2AS4WD)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("2ASxxx", "2AS4WD");
  std::string sdf_description = make_sdf_description("2AS4WD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface2ASxxx gazebo_interface(GetModel("robot"), hardware_info[0], "velocity");

  romea::SimulationCommand2ASxxx command = {0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 2.0, -2.0, 3.0, -3.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(
    command.frontAxleSteeringAngle,
    state.frontAxleSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearAxleSteeringAngle,
    state.rearAxleSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSteeringAngle,
    state.frontLeftWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.frontRightWheelSteeringAngle,
    state.frontRightWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearLeftWheelSteeringAngle,
    state.rearLeftWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearRightWheelSteeringAngle,
    state.rearRightWheelSteeringAngle,
    0.1);
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


TEST_F(TestGazeboInterface2ASxxx, testSetGet2AS2FWD)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("2ASxxx", "2AS2FWD");
  std::string sdf_description = make_sdf_description("2AS2FWD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface2ASxxx gazebo_interface(GetModel("robot"), hardware_info[0], "velocity");

  romea::SimulationCommand2ASxxx command = {0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 2.0, -2.0, 3.0, -3.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(
    command.frontAxleSteeringAngle,
    state.frontAxleSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearAxleSteeringAngle,
    state.rearAxleSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSteeringAngle,
    state.frontLeftWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.frontRightWheelSteeringAngle,
    state.frontRightWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearLeftWheelSteeringAngle,
    state.rearLeftWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearRightWheelSteeringAngle,
    state.rearRightWheelSteeringAngle,
    0.1);
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

TEST_F(TestGazeboInterface2ASxxx, testSetGet2AS2RWD)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("2ASxxx", "2AS2RWD");
  std::string sdf_description = make_sdf_description("2AS2RWD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface2ASxxx gazebo_interface(GetModel("robot"), hardware_info[0], "velocity");

  romea::SimulationCommand2ASxxx command = {0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 2.0, -2.0, 3.0, -3.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(
    command.frontAxleSteeringAngle,
    state.frontAxleSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearAxleSteeringAngle,
    state.rearAxleSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.frontLeftWheelSteeringAngle,
    state.frontLeftWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.frontRightWheelSteeringAngle,
    state.frontRightWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearLeftWheelSteeringAngle,
    state.rearLeftWheelSteeringAngle,
    0.1);
  EXPECT_NEAR(
    command.rearRightWheelSteeringAngle,
    state.rearRightWheelSteeringAngle,
    0.1);
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
