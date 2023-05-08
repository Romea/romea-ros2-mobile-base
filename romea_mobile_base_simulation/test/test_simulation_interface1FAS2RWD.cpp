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
#include "romea_mobile_base_simulation/simulation_interface1FAS2RWD.hpp"

class TestSimulationInterface1FAS2RWD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_simulation_interface1FAS2RWD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface1FAS2RWD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::SimulationInterface1FAS2RWD>(info[0], "velocity");
  }

  std::unique_ptr<romea::SimulationInterface1FAS2RWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestSimulationInterface1FAS2RWD, checkSetCommand)
{
  romea::HardwareCommand1FAS2RWD command = {-0.5, -1.83079, -1.02635};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontAxleSteeringAngle);
  command_interfaces[1].set_value(command.rearLeftWheelSpinningSetPoint);
  command_interfaces[2].set_value(command.rearRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  EXPECT_NEAR(simulation_command.frontAxleSteeringAngle, -0.5, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftWheelSteeringAngle, -0.437465, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSteeringAngle, -0.581062, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftWheelSpinningSetPoint, -4.4105, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSpinningSetPoint, -3.20543, 0.001);
  EXPECT_NEAR(simulation_command.rearLeftWheelSpinningSetPoint, -1.83079, 0.001);
  EXPECT_NEAR(simulation_command.rearRightWheelSpinningSetPoint, -1.02635, 0.001);
}

TEST_F(TestSimulationInterface1FAS2RWD, checkGetState)
{
  romea::HardwareCommand1FAS2RWD command = {0.3, 2.2471, 2.99038};
  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontAxleSteeringAngle);
  command_interfaces[1].set_value(command.rearLeftWheelSpinningSetPoint);
  command_interfaces[2].set_value(command.rearRightWheelSpinningSetPoint);

  romea::SimulationCommand1FASxxx simulation_command = interface->get_command();

  romea::SimulationState1FASxxx simulation_state;
  simulation_state.frontAxleSteeringAngle =
    simulation_command.frontAxleSteeringAngle;
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
  interface->set_state(simulation_state);

  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_NEAR(
    state_interfaces[0].get_value(),
    command.frontAxleSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[2].get_value(),
    command.rearLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[5].get_value(),
    command.rearRightWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[7].get_value(),
    simulation_command.frontLeftWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[8].get_value(),
    simulation_command.frontRightWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[10].get_value(),
    simulation_command.frontLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[13].get_value(),
    simulation_command.frontRightWheelSpinningSetPoint,
    0.001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
