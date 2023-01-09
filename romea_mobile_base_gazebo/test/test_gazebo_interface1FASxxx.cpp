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
#include "romea_mobile_base_gazebo/gazebo_interface1FASxxx.hpp"

class TestGazeboInterface1FASxxx : public gazebo::ServerFixture {};


TEST_F(TestGazeboInterface1FASxxx, testSetGet1FAS2FWD)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("1FASxxx", "1FAS2FWD");
  std::string sdf_description = make_sdf_description("1FAS2FWD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface1FASxxx gazebo_interface(GetModel("robot"), hardware_info[0], "velocity");

  romea::SimulationCommand1FASxxx command = {0.1, 0.2, 0.3, 2.0, -2.0, 3.0, -3.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(command.frontAxleSteeringAngle, state.frontAxleSteeringAngle, 0.1);
  EXPECT_NEAR(command.frontLeftWheelSteeringAngle, state.frontLeftWheelSteeringAngle, 0.1);
  EXPECT_NEAR(command.frontRightWheelSteeringAngle, state.frontRightWheelSteeringAngle, 0.1);
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

TEST_F(TestGazeboInterface1FASxxx, testSetGet1FAS2RWD)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("1FASxxx", "1FAS2RWD");
  std::string sdf_description = make_sdf_description("1FAS2RWD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface1FASxxx gazebo_interface(GetModel("robot"), hardware_info[0], "velocity");

  romea::SimulationCommand1FASxxx command = {0.1, 0.2, 0.3, 2.0, -2.0, 3.0, -3.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(
    command.frontAxleSteeringAngle,
    state.frontAxleSteeringAngle,
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

TEST_F(TestGazeboInterface1FASxxx, testSetGet1FAS4WD)
{
  Load("/usr/share/gazebo-11/worlds/empty.world");
  std::string urdf_description = make_urdf_description("1FASxxx", "1FAS4WD");
  std::string sdf_description = make_sdf_description("1FAS4WD");
  SpawnSDF(sdf_description);

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_description);
  romea::GazeboInterface1FASxxx gazebo_interface(GetModel("robot"), hardware_info[0], "velocity");

  romea::SimulationCommand1FASxxx command = {0.1, 0.2, 0.3, 2.0, -2.0, 3.0, -3.0};
  gazebo_interface.set_command(command);
  auto state = gazebo_interface.get_state();

  EXPECT_NEAR(
    command.frontAxleSteeringAngle,
    state.frontAxleSteeringAngle,
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
