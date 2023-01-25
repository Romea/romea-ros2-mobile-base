// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


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
#include "romea_mobile_base_simulation/simulation_interface2TTD.hpp"

class TestSimulationInterface2TTD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_simulation_interface2TTD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface2TTD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::SimulationInterface2TTD>(info[0], "velocity");
  }

  std::unique_ptr<romea::SimulationInterface2TTD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestSimulationInterface2TTD, checkGetCommand)
{
  romea::HardwareCommand2TD command = {0.611111, 1.6111};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.leftSprocketWheelSpinningSetPoint);
  command_interfaces[1].set_value(command.rightSprocketWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  EXPECT_NEAR(
    simulation_command.leftSprocketWheelSpinningSetPoint,
    command.leftSprocketWheelSpinningSetPoint, 0.001);
  EXPECT_NEAR(
    simulation_command.rightSprocketWheelSpinningSetPoint,
    command.rightSprocketWheelSpinningSetPoint, 0.001);
  EXPECT_NEAR(simulation_command.leftIdlerWheelSpinningSetPoint, 1.375, 0.001);
  EXPECT_NEAR(simulation_command.rightIdlerWheelSpinningSetPoint, 3.625, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftRollerWheelSpinningSetPoint, 1.375, 0.001);
  EXPECT_NEAR(simulation_command.frontRightRollerWheelSpinningSetPoint, 3.625, 0.001);
  EXPECT_NEAR(simulation_command.rearLeftRollerWheelSpinningSetPoint, 1.375, 0.001);
  EXPECT_NEAR(simulation_command.rearRightRollerWheelSpinningSetPoint, 3.625, 0.001);
}

TEST_F(TestSimulationInterface2TTD, checkGetState)
{
  romea::HardwareCommand2TD command = {0.611111, 1.6111};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.leftSprocketWheelSpinningSetPoint);
  command_interfaces[1].set_value(command.rightSprocketWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  romea::SimulationState2TTD simulation_state;
  simulation_state.leftSprocketWheelSpinningMotion.velocity =
    simulation_command.leftSprocketWheelSpinningSetPoint;
  simulation_state.rightSprocketWheelSpinningMotion.velocity =
    simulation_command.rightSprocketWheelSpinningSetPoint;
  simulation_state.leftIdlerWheelSpinningMotion.velocity =
    simulation_command.leftIdlerWheelSpinningSetPoint;
  simulation_state.rightIdlerWheelSpinningMotion.velocity =
    simulation_command.rightIdlerWheelSpinningSetPoint;
  simulation_state.frontLeftRollerWheelSpinningMotion.velocity =
    simulation_command.frontLeftRollerWheelSpinningSetPoint;
  simulation_state.frontRightRollerWheelSpinningMotion.velocity =
    simulation_command.frontRightRollerWheelSpinningSetPoint;
  simulation_state.rearLeftRollerWheelSpinningMotion.velocity =
    simulation_command.rearLeftRollerWheelSpinningSetPoint;
  simulation_state.rearRightRollerWheelSpinningMotion.velocity =
    simulation_command.rearRightRollerWheelSpinningSetPoint;
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
  EXPECT_NEAR(
    state_interfaces[13].get_value(),
    simulation_command.frontLeftRollerWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[16].get_value(),
    simulation_command.frontRightRollerWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[19].get_value(),
    simulation_command.rearLeftRollerWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[22].get_value(),
    simulation_command.rearRightRollerWheelSpinningSetPoint,
    0.001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
