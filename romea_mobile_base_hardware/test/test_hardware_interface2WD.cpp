// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// gtest
#include <gtest/gtest.h>

// ros
#include <rclcpp/node.hpp>
#include <hardware_interface/component_parser.hpp>

// std
#include <memory>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

// local
#include "../test/test_helper.h"
#include "romea_mobile_base_hardware/hardware_interface2WD.hpp"

class TestHarwareInterface2WD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface2WD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface2WD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface2WD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2WD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(), "robot_joint1");
  EXPECT_STREQ(state_interfaces[3].get_name().c_str(), "robot_joint2");
}

TEST_F(TestHarwareInterface2WD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/velocity");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/velocity");
}

TEST_F(TestHarwareInterface2WD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/effort");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/effort");
}

TEST_F(TestHarwareInterface2WD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2WD current_state;
  current_state.leftWheelSpinningMotion.position = 1.0;
  current_state.leftWheelSpinningMotion.velocity = 2.0;
  current_state.leftWheelSpinningMotion.torque = 3.0;
  current_state.rightWheelSpinningMotion.position = 4.0;
  current_state.rightWheelSpinningMotion.velocity = 5.0;
  current_state.rightWheelSpinningMotion.torque = 6.0;

  interface->set_state(current_state);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2WD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 2; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::HardwareCommand2WD current_command = interface->get_command();
  EXPECT_DOUBLE_EQ(current_command.leftWheelSpinningSetPoint, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rightWheelSpinningSetPoint, 2.0);
}
