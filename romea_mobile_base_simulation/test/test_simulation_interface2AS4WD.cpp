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
#include <memory>
#include <string>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"
#include "hardware_interface/component_parser.hpp"


// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_simulation/simulation_interface2AS4WD.hpp"

class TestSimulationInterface2AS4WD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_simulation_interface2AS4WD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface2AS4WD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::ros2::SimulationInterface2AS4WD>(info[0], "velocity");
  }

  std::unique_ptr<romea::ros2::SimulationInterface2AS4WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestSimulationInterface2AS4WD, checkGetCommand)
{
  romea::core::HardwareCommand2AS4WD command = {0.3, -0.3, 2.2471, 2.99038, 2.2471, 2.99038};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontAxleSteeringAngle);
  command_interfaces[1].set_value(command.rearAxleSteeringAngle);
  command_interfaces[2].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.frontRightWheelSpinningSetPoint);
  command_interfaces[4].set_value(command.rearLeftWheelSpinningSetPoint);
  command_interfaces[5].set_value(command.rearRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_hardware_command();

  EXPECT_NEAR(simulation_command.frontAxleSteeringAngle, 0.3, 0.001);
  EXPECT_NEAR(simulation_command.rearAxleSteeringAngle, -0.3, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftWheelSteeringAngle, 0.346313, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSteeringAngle, 0.264355, 0.001);
  EXPECT_NEAR(simulation_command.rearLeftWheelSteeringAngle, -0.264355, 0.001);
  EXPECT_NEAR(simulation_command.rearRightWheelSteeringAngle, -0.346313, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftWheelSpinningSetPoint, 2.2471, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSpinningSetPoint, 2.99038, 0.001);
  EXPECT_NEAR(simulation_command.rearLeftWheelSpinningSetPoint, 2.2471, 0.001);
  EXPECT_NEAR(simulation_command.rearRightWheelSpinningSetPoint, 2.99038, 0.001);
}

TEST_F(TestSimulationInterface2AS4WD, checkGetCommandUsingJointState)
{
  romea::core::HardwareCommand2AS4WD command = {0.3, -0.3, 2.2471, 2.99038, 2.2471, 2.99038};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontAxleSteeringAngle);
  command_interfaces[1].set_value(command.rearAxleSteeringAngle);
  command_interfaces[2].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.frontRightWheelSpinningSetPoint);
  command_interfaces[4].set_value(command.rearLeftWheelSpinningSetPoint);
  command_interfaces[5].set_value(command.rearRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_joint_state_command();

  EXPECT_STREQ(simulation_command.name[0].c_str(), "robot_joint1");
  EXPECT_STREQ(simulation_command.name[1].c_str(), "robot_joint2");
  EXPECT_STREQ(simulation_command.name[2].c_str(), "robot_joint3");
  EXPECT_STREQ(simulation_command.name[3].c_str(), "robot_joint4");
  EXPECT_STREQ(simulation_command.name[4].c_str(), "robot_joint5");
  EXPECT_STREQ(simulation_command.name[5].c_str(), "robot_joint6");
  EXPECT_STREQ(simulation_command.name[6].c_str(), "robot_joint7");
  EXPECT_STREQ(simulation_command.name[7].c_str(), "robot_joint8");
  EXPECT_STREQ(simulation_command.name[8].c_str(), "robot_joint9");
  EXPECT_STREQ(simulation_command.name[9].c_str(), "robot_joint10");

  EXPECT_NEAR(simulation_command.position[0], 0.3, 0.001);
  EXPECT_NEAR(simulation_command.position[1], -0.3, 0.001);
  EXPECT_NEAR(simulation_command.position[2], 0.346313, 0.001);
  EXPECT_NEAR(simulation_command.position[3], 0.264355, 0.001);
  EXPECT_NEAR(simulation_command.position[4], -0.264355, 0.001);
  EXPECT_NEAR(simulation_command.position[5], -0.346313, 0.001);
  EXPECT_NEAR(simulation_command.velocity[6], 2.2471, 0.001);
  EXPECT_NEAR(simulation_command.velocity[7], 2.99038, 0.001);
  EXPECT_NEAR(simulation_command.velocity[8], 2.2471, 0.001);
  EXPECT_NEAR(simulation_command.velocity[9], 2.99038, 0.001);
}


TEST_F(TestSimulationInterface2AS4WD, checkGetState)
{
  romea::core::HardwareCommand2AS4WD command = {0.3, -0.3, 2.2471, 2.99038, 2.2471, 2.99038};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontAxleSteeringAngle);
  command_interfaces[1].set_value(command.rearAxleSteeringAngle);
  command_interfaces[2].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.frontRightWheelSpinningSetPoint);
  command_interfaces[4].set_value(command.rearLeftWheelSpinningSetPoint);
  command_interfaces[5].set_value(command.rearRightWheelSpinningSetPoint);
  romea::core::SimulationCommand2AS4WD simulation_command = interface->get_hardware_command();

  romea::core::SimulationState2AS4WD simulation_state;
  simulation_state.frontAxleSteeringAngle =
    simulation_command.frontAxleSteeringAngle;
  simulation_state.rearAxleSteeringAngle =
    simulation_command.rearAxleSteeringAngle;
  simulation_state.frontLeftWheelSteeringAngle =
    simulation_command.frontLeftWheelSteeringAngle;
  simulation_state.frontRightWheelSteeringAngle =
    simulation_command.frontRightWheelSteeringAngle;
  simulation_state.rearLeftWheelSteeringAngle =
    simulation_command.rearLeftWheelSteeringAngle;
  simulation_state.rearRightWheelSteeringAngle =
    simulation_command.rearRightWheelSteeringAngle;
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
    command.frontAxleSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[1].get_value(),
    command.rearAxleSteeringAngle,
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
    command.rearLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[12].get_value(),
    command.rearRightWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[14].get_value(),
    simulation_command.frontLeftWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[15].get_value(),
    simulation_command.frontRightWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[16].get_value(),
    simulation_command.rearLeftWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[17].get_value(),
    simulation_command.rearRightWheelSteeringAngle,
    0.001);
}

TEST_F(TestSimulationInterface2AS4WD, checkGetStateUsingJointState)
{
  romea::core::HardwareCommand2AS4WD command = {0.3, -0.3, 2.2471, 2.99038, 2.2471, 2.99038};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontAxleSteeringAngle);
  command_interfaces[1].set_value(command.rearAxleSteeringAngle);
  command_interfaces[2].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.frontRightWheelSpinningSetPoint);
  command_interfaces[4].set_value(command.rearLeftWheelSpinningSetPoint);
  command_interfaces[5].set_value(command.rearRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_joint_state_command();
  interface->set_feedback(simulation_command);

  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_NEAR(state_interfaces[0].get_value(), simulation_command.position[0], 0.001);
  EXPECT_NEAR(state_interfaces[1].get_value(), simulation_command.position[1], 0.001);
  EXPECT_NEAR(state_interfaces[3].get_value(), simulation_command.velocity[6], 0.001);
  EXPECT_NEAR(state_interfaces[6].get_value(), simulation_command.velocity[7], 0.001);
  EXPECT_NEAR(state_interfaces[9].get_value(), simulation_command.velocity[8], 0.001);
  EXPECT_NEAR(state_interfaces[12].get_value(), simulation_command.velocity[9], 0.001);
  EXPECT_NEAR(state_interfaces[14].get_value(), simulation_command.position[2], 0.001);
  EXPECT_NEAR(state_interfaces[15].get_value(), simulation_command.position[3], 0.001);
  EXPECT_NEAR(state_interfaces[16].get_value(), simulation_command.position[4], 0.001);
  EXPECT_NEAR(state_interfaces[17].get_value(), simulation_command.position[5], 0.001);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
