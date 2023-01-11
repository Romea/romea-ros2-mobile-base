// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// ros
#include <rclcpp/node.hpp>
#include <hardware_interface/component_parser.hpp>

// std
#include <fstream>
#include <memory>
#include <string>
#include <vector>


// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_simulation/simulation_interface2FWS2RWD.hpp"

class TestSimulationInterface2FWS2RWD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_simulation_interface2FWS2RWD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface2FWS2RWD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::SimulationInterface2FWS2RWD>(info[0], "velocity");
  }

  std::unique_ptr<romea::SimulationInterface2FWS2RWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestSimulationInterface2FWS2RWD, checkSetCommand)
{
  romea::HardwareCommand2FWS2RWD command = {-0.392757, -0.675853, 4.16796, 2.4987};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontLeftWheelSteeringAngle);
  command_interfaces[1].set_value(command.frontRightWheelSteeringAngle);
  command_interfaces[2].set_value(command.rearLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.rearRightWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  std::cout << simulation_command << std::endl;
  EXPECT_NEAR(simulation_command.frontLeftWheelSteeringAngle, -0.392757, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSteeringAngle, -0.675853, 0.001);
  EXPECT_NEAR(simulation_command.frontLeftWheelSpinningSetPoint, 2.94577, 0.001);
  EXPECT_NEAR(simulation_command.frontRightWheelSpinningSetPoint, 1.65554, 0.001);
  EXPECT_NEAR(simulation_command.rearLeftWheelSpinningSetPoint, 4.16796, 0.001);
  EXPECT_NEAR(simulation_command.rearRightWheelSpinningSetPoint, 2.4987, 0.001);
}

TEST_F(TestSimulationInterface2FWS2RWD, checkGetState)
{
  romea::HardwareCommand2FWS2RWD command = {-0.392757, -0.675853, 4.16796, 2.4987};
  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.frontLeftWheelSteeringAngle);
  command_interfaces[1].set_value(command.frontRightWheelSteeringAngle);
  command_interfaces[2].set_value(command.rearLeftWheelSpinningSetPoint);
  command_interfaces[3].set_value(command.rearRightWheelSpinningSetPoint);

  romea::SimulationCommand2FWSxxx simulation_command = interface->get_command();

  romea::SimulationState2FWSxxx simulation_state;
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
    command.frontLeftWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[1].get_value(),
    command.frontRightWheelSteeringAngle,
    0.001);
  EXPECT_NEAR(
    state_interfaces[3].get_value(),
    command.rearLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[6].get_value(),
    command.rearRightWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[9].get_value(),
    simulation_command.frontLeftWheelSpinningSetPoint,
    0.001);
  EXPECT_NEAR(
    state_interfaces[12].get_value(),
    simulation_command.frontRightWheelSpinningSetPoint,
    0.001);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
