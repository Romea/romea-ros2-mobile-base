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
#include "romea_mobile_base_simulation/simulation_interface2TD.hpp"

class TestSimulationInterface2TD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_simulation_interface2TD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface2TD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::ros2::SimulationInterface2TD>(info[0], "velocity");
  }

  std::unique_ptr<romea::ros2::SimulationInterface2TD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestSimulationInterface2TD, checkGetCommand)
{
  romea::core::HardwareCommand2TD command = {0.611111, 1.6111};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.leftSprocketWheelSpinningSetPoint);
  command_interfaces[1].set_value(command.rightSprocketWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  EXPECT_NEAR(
    simulation_command.leftSprocketWheelSpinningSetPoint,
    command.leftSprocketWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    simulation_command.rightSprocketWheelSpinningSetPoint,
    command.rightSprocketWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(simulation_command.leftIdlerWheelSpinningSetPoint, 1.375, 0.001);
  EXPECT_NEAR(simulation_command.rightIdlerWheelSpinningSetPoint, 3.625, 0.001);
}

TEST_F(TestSimulationInterface2TD, checkGetState)
{
  romea::core::HardwareCommand2TD command = {0.611111, 1.6111};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.leftSprocketWheelSpinningSetPoint);
  command_interfaces[1].set_value(command.rightSprocketWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  romea::core::SimulationState2TD simulation_state;
  simulation_state.leftSprocketWheelSpinningMotion.velocity =
    simulation_command.leftSprocketWheelSpinningSetPoint;
  simulation_state.rightSprocketWheelSpinningMotion.velocity =
    simulation_command.rightSprocketWheelSpinningSetPoint;
  simulation_state.leftIdlerWheelSpinningMotion.velocity =
    simulation_command.leftIdlerWheelSpinningSetPoint;
  simulation_state.rightIdlerWheelSpinningMotion.velocity =
    simulation_command.rightIdlerWheelSpinningSetPoint;
  interface->set_state(simulation_state);

  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_NEAR(
    state_interfaces[1].get_value(),
    simulation_command.leftSprocketWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[4].get_value(),
    simulation_command.rightSprocketWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[7].get_value(),
    simulation_command.leftIdlerWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[10].get_value(),
    simulation_command.rightIdlerWheelSpinningSetPoint,
    0.001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
