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

// local
#include "../test/test_helper.h"
#include "romea_mobile_base_hardware/hardware_interface4WS4WD.hpp"

class TestHarwareInterface4WS4WD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface4WS4WD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface4WS4WD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface4WS4WD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface4WS4WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface4WS4WD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(), "robot_joint1");
  EXPECT_STREQ(state_interfaces[1].get_name().c_str(), "robot_joint2");
  EXPECT_STREQ(state_interfaces[2].get_name().c_str(), "robot_joint3");
  EXPECT_STREQ(state_interfaces[3].get_name().c_str(), "robot_joint4");
  EXPECT_STREQ(state_interfaces[4].get_name().c_str(), "robot_joint5");
  EXPECT_STREQ(state_interfaces[7].get_name().c_str(), "robot_joint6");
  EXPECT_STREQ(state_interfaces[10].get_name().c_str(), "robot_joint7");
  EXPECT_STREQ(state_interfaces[13].get_name().c_str(), "robot_joint8");
}

TEST_F(TestHarwareInterface4WS4WD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/position");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(), "robot_joint3/position");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(), "robot_joint4/position");
  EXPECT_STREQ(command_interfaces[4].get_full_name().c_str(), "robot_joint5/velocity");
  EXPECT_STREQ(command_interfaces[5].get_full_name().c_str(), "robot_joint6/velocity");
  EXPECT_STREQ(command_interfaces[6].get_full_name().c_str(), "robot_joint7/velocity");
  EXPECT_STREQ(command_interfaces[7].get_full_name().c_str(), "robot_joint8/velocity");
}

TEST_F(TestHarwareInterface4WS4WD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/position");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(), "robot_joint3/position");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(), "robot_joint4/position");
  EXPECT_STREQ(command_interfaces[4].get_full_name().c_str(), "robot_joint5/effort");
  EXPECT_STREQ(command_interfaces[5].get_full_name().c_str(), "robot_joint6/effort");
  EXPECT_STREQ(command_interfaces[6].get_full_name().c_str(), "robot_joint7/effort");
  EXPECT_STREQ(command_interfaces[7].get_full_name().c_str(), "robot_joint8/effort");
}


TEST_F(TestHarwareInterface4WS4WD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState4WS4WD current_state;
  current_state.frontLeftWheelSteeringAngle = 1.0;
  current_state.frontRightWheelSteeringAngle = 2.0;
  current_state.rearLeftWheelSteeringAngle = 3.0;
  current_state.rearRightWheelSteeringAngle = 4.0;
  current_state.frontLeftWheelSpinningMotion.position = 5.0;
  current_state.frontLeftWheelSpinningMotion.velocity = 6.0;
  current_state.frontLeftWheelSpinningMotion.torque = 7.0;
  current_state.frontRightWheelSpinningMotion.position = 8.0;
  current_state.frontRightWheelSpinningMotion.velocity = 9.0;
  current_state.frontRightWheelSpinningMotion.torque = 10.0;
  current_state.rearLeftWheelSpinningMotion.position = 11.0;
  current_state.rearLeftWheelSpinningMotion.velocity = 12.0;
  current_state.rearLeftWheelSpinningMotion.torque = 13.0;
  current_state.rearRightWheelSpinningMotion.position = 14.0;
  current_state.rearRightWheelSpinningMotion.velocity = 15.0;
  current_state.rearRightWheelSpinningMotion.torque = 16.0;

  interface->set_state(current_state);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 16; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface4WS4WD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 8; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::HardwareCommand4WS4WD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSteeringAngle, 3.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSteeringAngle, 4.0);
  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSpinningSetPoint, 5.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSpinningSetPoint, 6.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSpinningSetPoint, 7.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSpinningSetPoint, 8.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
