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
#include "romea_mobile_base_simulation/simulation_interface1FAS2FWD.hpp"

class TestSimulationInterface1FAS2FWD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_simulation_interface1FAS2FWD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface1FAS2FWD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::SimulationInterface1FAS2FWD>(info[0], "velocity");
  }

  std::unique_ptr<romea::SimulationInterface1FAS2FWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestSimulationInterface1FAS2FWD, checkSetCommand)
{
  romea::HardwareCommand1FAS2FWD command = {0.3, 2.2471, 2.99038};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontAxleSteeringAngle);
  command_interfaces[1].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[2].set_value(command.frontRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  EXPECT_NEAR(simulation_command.frontAxleSteeringAngle, 0.3, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftWheelSteeringAngle, 0.342554, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSteeringAngle, 0.266624, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftWheelSpinningSetPoint, 2.2471, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSpinningSetPoint, 2.99038, 0.001);
  EXPECT_NEAR(simulation_command.rearLeftWheelSpinningSetPoint, 1.35365, 0.001);
  EXPECT_NEAR(simulation_command.rearRightWheelSpinningSetPoint, 1.97969, 0.001);
}

TEST_F(TestSimulationInterface1FAS2FWD, checkGetState)
{
  romea::HardwareCommand1FAS2FWD command = {0.3, 2.2471, 2.99038};
  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontAxleSteeringAngle);
  command_interfaces[1].set_value(command.frontLeftWheelSpinningSetPoint);
  command_interfaces[2].set_value(command.frontRightWheelSpinningSetPoint);

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
    command.frontLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[5].get_value(),
    command.frontRightWheelSpinningSetPoint,
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
    simulation_command.rearLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[13].get_value(),
    simulation_command.rearRightWheelSpinningSetPoint,
    0.001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
