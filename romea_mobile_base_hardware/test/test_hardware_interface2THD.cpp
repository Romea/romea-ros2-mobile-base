// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// std
#include <fstream>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"
#include "hardware_interface/component_parser.hpp"


// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_hardware/hardware_interface2THD.hpp"

class TestHarwareInterface2THD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface2THD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface2THD.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    //    std::cout << buffer.str() <<std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
  }

  void MakeInterface(const std::string & command_interface_type)
  {
    interface = std::make_unique<romea::HardwareInterface2THD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2THD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2THD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(), "robot_joint1");
  EXPECT_STREQ(state_interfaces[3].get_name().c_str(), "robot_joint2");
  EXPECT_STREQ(state_interfaces[6].get_name().c_str(), "robot_joint3");
  EXPECT_STREQ(state_interfaces[9].get_name().c_str(), "robot_joint4");
  EXPECT_STREQ(state_interfaces[12].get_name().c_str(), "robot_joint5");
  EXPECT_STREQ(state_interfaces[15].get_name().c_str(), "robot_joint6");
}

TEST_F(TestHarwareInterface2THD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/velocity");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/velocity");
}

TEST_F(TestHarwareInterface2THD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/effort");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/effort");
}

TEST_F(TestHarwareInterface2THD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2TD current_state;
  current_state.leftSprocketWheelSpinningMotion.position = 1.0;
  current_state.leftSprocketWheelSpinningMotion.velocity = 2.0;
  current_state.leftSprocketWheelSpinningMotion.torque = 3.0;
  current_state.rightSprocketWheelSpinningMotion.position = 4.0;
  current_state.rightSprocketWheelSpinningMotion.velocity = 5.0;
  current_state.rightSprocketWheelSpinningMotion.torque = 6.0;
  romea::RotationalMotionState frontLeftIdlerWheelSpinningMotion;
  frontLeftIdlerWheelSpinningMotion.position = 7.0;
  frontLeftIdlerWheelSpinningMotion.velocity = 8.0;
  frontLeftIdlerWheelSpinningMotion.torque = 9.0;
  romea::RotationalMotionState frontRightIdlerWheelSpinningMotion;
  frontRightIdlerWheelSpinningMotion.position = 10.0;
  frontRightIdlerWheelSpinningMotion.velocity = 11.0;
  frontRightIdlerWheelSpinningMotion.torque = 12.0;
  romea::RotationalMotionState rearLeftIdlerWheelSpinningMotion;
  rearLeftIdlerWheelSpinningMotion.position = 13.0;
  rearLeftIdlerWheelSpinningMotion.velocity = 14.0;
  rearLeftIdlerWheelSpinningMotion.torque = 15.0;
  romea::RotationalMotionState rearRightIdlerWheelSpinningMotion;
  rearRightIdlerWheelSpinningMotion.position = 16.0;
  rearRightIdlerWheelSpinningMotion.velocity = 17.0;
  rearRightIdlerWheelSpinningMotion.torque = 18.0;

  interface->set_state(
    current_state,
    frontLeftIdlerWheelSpinningMotion,
    frontRightIdlerWheelSpinningMotion,
    rearLeftIdlerWheelSpinningMotion,
    rearRightIdlerWheelSpinningMotion);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 12; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2THD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 2; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::HardwareCommand2TD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.leftSprocketWheelSpinningSetPoint, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rightSprocketWheelSpinningSetPoint, 2.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
