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
#include <fstream>
#include <string>
#include <memory>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"
#include "hardware_interface/component_parser.hpp"

// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_simulation/simulation_interface2FWS2FWD.hpp"

class TestSimulationInterface2FWS2FWD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_simulation_interface2FWS2FWD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface2FWS2FWD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::ros2::SimulationInterface2FWS2FWD>(info[0], "velocity");
  }

  std::unique_ptr<romea::ros2::SimulationInterface2FWS2FWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestSimulationInterface2FWS2FWD, checkGetCommand)
{
  romea::core::HardwareCommand2FWS2FWD command = {0.476646, 0.343727, 2.22829, 3.21196};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontLeftWheelSteeringAngle);
  command_interfaces[1].set_value(command.frontRightWheelSteeringAngle);
  command_interfaces[2].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.frontRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_hardware_command();

  EXPECT_NEAR(simulation_command.frontLeftWheelSteeringAngle, 0.476646, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSteeringAngle, 0.343727, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftWheelSpinningSetPoint, 2.22829, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSpinningSetPoint, 3.21196, 0.001);
  EXPECT_NEAR(simulation_command.rearLeftWheelSpinningSetPoint, 1.23884, 0.001);
  EXPECT_NEAR(simulation_command.rearRightWheelSpinningSetPoint, 2.09449, 0.001);
}


TEST_F(TestSimulationInterface2FWS2FWD, checkGetCommandUsingJointState)
{
  romea::core::HardwareCommand2FWS2FWD command = {0.476646, 0.343727, 2.22829, 3.21196};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontLeftWheelSteeringAngle);
  command_interfaces[1].set_value(command.frontRightWheelSteeringAngle);
  command_interfaces[2].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.frontRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_joint_state_command();

  EXPECT_STREQ(simulation_command.name[0].c_str(), "robot_joint1");
  EXPECT_STREQ(simulation_command.name[1].c_str(), "robot_joint2");
  EXPECT_STREQ(simulation_command.name[2].c_str(), "robot_joint3");
  EXPECT_STREQ(simulation_command.name[3].c_str(), "robot_joint4");
  EXPECT_STREQ(simulation_command.name[4].c_str(), "robot_joint5");
  EXPECT_STREQ(simulation_command.name[5].c_str(), "robot_joint6");

  EXPECT_NEAR(simulation_command.position[0], 0.476646, 0.001);
  EXPECT_NEAR(simulation_command.position[1], 0.343727, 0.001);
  EXPECT_NEAR(simulation_command.velocity[2], 2.22829, 0.001);
  EXPECT_NEAR(simulation_command.velocity[3], 3.21196, 0.001);
  EXPECT_NEAR(simulation_command.velocity[4], 1.23884, 0.001);
  EXPECT_NEAR(simulation_command.velocity[5], 2.09449, 0.001);
}

TEST_F(TestSimulationInterface2FWS2FWD, checkGetState)
{
  romea::core::HardwareCommand2FWS2FWD command = {0.476646, 0.343727, 2.22829, 3.21196};
  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontLeftWheelSteeringAngle);
  command_interfaces[1].set_value(command.frontRightWheelSteeringAngle);
  command_interfaces[2].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.frontRightWheelSpinningSetPoint);

  romea::core::SimulationCommand2FWSxxx simulation_command = interface->get_hardware_command();

  romea::core::SimulationState2FWSxxx simulation_state;
  simulation_state.frontLeftWheelSteeringAngle =
    simulation_command.frontLeftWheelSteeringAngle;
  simulation_state.frontRightWheelSteeringAngle =
    simulation_command.frontRightWheelSteeringAngle;
  simulation_state.frontLeftWheelSpinningMotion.velocity =
    simulation_command.frontLeftWheelSpinningSetPoint;
  simulation_state.frontRightWheelSpinningMotion.velocity =
    simulation_command.frontRightWheelSpinningSetPoint;
  simulation_state.rearLeftWheelSpinningMotion.velocity =
    simulation_command.rearLeftWheelSpinningSetPoint;
  simulation_state.rearRightWheelSpinningMotion.velocity =
    simulation_command.rearRightWheelSpinningSetPoint;
  interface->set_feedback(simulation_state);

  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_NEAR(
    state_interfaces[0].get_value(),
    command.frontLeftWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[1].get_value(),
    command.frontRightWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[3].get_value(),
    command.frontLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[6].get_value(),
    command.frontRightWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[9].get_value(),
    simulation_command.rearLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[12].get_value(),
    simulation_command.rearRightWheelSpinningSetPoint,
    0.001);
}

TEST_F(TestSimulationInterface2FWS2FWD, checkGetStateUsingJointState)
{
  romea::core::HardwareCommand2FWS2FWD command = {0.476646, 0.343727, 2.22829, 3.21196};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontLeftWheelSteeringAngle);
  command_interfaces[1].set_value(command.frontRightWheelSteeringAngle);
  command_interfaces[2].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.frontRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_joint_state_command();
  interface->set_feedback(simulation_command);

  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_NEAR(state_interfaces[0].get_value(), simulation_command.position[0], 0.001);
  EXPECT_NEAR(state_interfaces[1].get_value(), simulation_command.position[1], 0.001);
  EXPECT_NEAR(state_interfaces[3].get_value(), simulation_command.velocity[2], 0.001);
  EXPECT_NEAR(state_interfaces[6].get_value(), simulation_command.velocity[3], 0.001);
  EXPECT_NEAR(state_interfaces[9].get_value(), simulation_command.velocity[4], 0.001);
  EXPECT_NEAR(state_interfaces[12].get_value(), simulation_command.velocity[5], 0.001);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
