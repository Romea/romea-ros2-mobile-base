//gtest
#include <gtest/gtest.h>
#include "test_helper.h"
#include <fstream>

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_simulation/simulation_interface2THD.hpp"
#include <hardware_interface/component_parser.hpp>

class TestSimulationInterface2THD : public ::testing::Test
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
    std::string xacro_file =  std::string(TEST_DIR)+"/test_simulation_interface2THD.xacro";
    std::string urdf_file =  "/tmp/test_simulation_interface2THD.urdf";
    std::string cmd = "xacro "+xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
    interface = std::make_unique<romea::SimulationInterface2THD>(info[0],"velocity");
  }

  std::unique_ptr<romea::SimulationInterface2THD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};




TEST_F(TestSimulationInterface2THD, checkGetCommand)
{
  romea::HardwareCommand2TD command = {0.611111,1.6111};

  auto command_interfaces   = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.leftSprocketWheelSpinningSetPoint);
  command_interfaces[1].set_value(command.rightSprocketWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  EXPECT_NEAR(simulation_command.leftSprocketWheelSpinningSetPoint,command.leftSprocketWheelSpinningSetPoint,0.001);
  EXPECT_NEAR(simulation_command.rightSprocketWheelSpinningSetPoint,command.rightSprocketWheelSpinningSetPoint,0.001);
  EXPECT_NEAR(simulation_command.frontLeftIdlerWheelSpinningSetPoint,1.375,0.001);
  EXPECT_NEAR(simulation_command.frontRightIdlerWheelSpinningSetPoint,3.625,0.001);
  EXPECT_NEAR(simulation_command.rearLeftIdlerWheelSpinningSetPoint,1.375,0.001);
  EXPECT_NEAR(simulation_command.rearRightIdlerWheelSpinningSetPoint,3.625,0.001);
}

TEST_F(TestSimulationInterface2THD, checkGetState)
{
  romea::HardwareCommand2TD command = {0.611111,1.6111};

  auto command_interfaces = interface->export_command_interfaces();
  command_interfaces[0].set_value(command.leftSprocketWheelSpinningSetPoint);
  command_interfaces[1].set_value(command.rightSprocketWheelSpinningSetPoint);
  auto simulation_command = interface->get_command();

  romea::SimulationState2THD simulation_state;
  simulation_state.leftSprocketWheelSpinningMotion.velocity=simulation_command.leftSprocketWheelSpinningSetPoint;
  simulation_state.rightSprocketWheelSpinningMotion.velocity=simulation_command.rightSprocketWheelSpinningSetPoint;
  simulation_state.frontLeftIdlerWheelSpinningMotion.velocity=simulation_command.frontLeftIdlerWheelSpinningSetPoint;
  simulation_state.frontRightIdlerWheelSpinningMotion.velocity=simulation_command.frontRightIdlerWheelSpinningSetPoint;
  simulation_state.rearLeftIdlerWheelSpinningMotion.velocity=simulation_command.rearLeftIdlerWheelSpinningSetPoint;
  simulation_state.rearRightIdlerWheelSpinningMotion.velocity=simulation_command.rearRightIdlerWheelSpinningSetPoint;
  interface->set_state(simulation_state);

  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_NEAR(state_interfaces[1].get_value(),simulation_command.leftSprocketWheelSpinningSetPoint,0.001);
  EXPECT_NEAR(state_interfaces[4].get_value(),simulation_command.rightSprocketWheelSpinningSetPoint,0.001);
  EXPECT_NEAR(state_interfaces[7].get_value(),simulation_command.frontLeftIdlerWheelSpinningSetPoint,0.001);
  EXPECT_NEAR(state_interfaces[10].get_value(),simulation_command.frontRightIdlerWheelSpinningSetPoint,0.001);
  EXPECT_NEAR(state_interfaces[13].get_value(),simulation_command.rearLeftIdlerWheelSpinningSetPoint,0.001);
  EXPECT_NEAR(state_interfaces[16].get_value(),simulation_command.rearRightIdlerWheelSpinningSetPoint,0.001);
}
