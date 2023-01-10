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
#include <sstream>
#include <vector>

// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_hardware/hardware_interface2FWS4WD.hpp"

class TestHarwareInterface2FWS4WD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface2FWS4WD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface2FWS4WD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface2FWS4WD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2FWS4WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2FWS4WD, checkJointNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(), "robot_joint1");
  EXPECT_STREQ(state_interfaces[1].get_name().c_str(), "robot_joint2");
  EXPECT_STREQ(state_interfaces[2].get_name().c_str(), "robot_joint3");
  EXPECT_STREQ(state_interfaces[5].get_name().c_str(), "robot_joint4");
  EXPECT_STREQ(state_interfaces[8].get_name().c_str(), "robot_joint5");
  EXPECT_STREQ(state_interfaces[11].get_name().c_str(), "robot_joint6");
}

// TEST_F(TestHarwareInterface2FWS4WD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
// {
//   MakeInterface(hardware_interface::HW_IF_VELOCITY);
//   EXPECT_STREQ(
//     interface->front_left_wheel_spinning_joint.command.get_interface_type().c_str(),
//     hardware_interface::HW_IF_VELOCITY);
//   EXPECT_STREQ(
//     interface->front_right_wheel_spinning_joint.command.get_interface_type().c_str(),
//     hardware_interface::HW_IF_VELOCITY);
//   EXPECT_STREQ(
//     interface->rear_left_wheel_spinning_joint.command.get_interface_type().c_str(),
//     hardware_interface::HW_IF_VELOCITY);
//   EXPECT_STREQ(
//     interface->rear_right_wheel_spinning_joint.command.get_interface_type().c_str(),
//     hardware_interface::HW_IF_VELOCITY);
// }

// TEST_F(TestHarwareInterface2FWS4WD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
// {
//   MakeInterface(hardware_interface::HW_IF_EFFORT);
//   EXPECT_STREQ(
//     interface->front_left_wheel_spinning_joint.command.get_interface_type().c_str(),
//     hardware_interface::HW_IF_EFFORT);
//   EXPECT_STREQ(
//     interface->front_right_wheel_spinning_joint.command.get_interface_type().c_str(),
//     hardware_interface::HW_IF_EFFORT);
//   EXPECT_STREQ(
//     interface->rear_left_wheel_spinning_joint.command.get_interface_type().c_str(),
//     hardware_interface::HW_IF_EFFORT);
//   EXPECT_STREQ(
//     interface->rear_right_wheel_spinning_joint.command.get_interface_type().c_str(),
//     hardware_interface::HW_IF_EFFORT);
// }

TEST_F(TestHarwareInterface2FWS4WD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2FWS4WD current_state;
  current_state.frontLeftWheelSteeringAngle = 1.0;
  current_state.frontRightWheelSteeringAngle = 2.0;
  current_state.frontLeftWheelSpinningMotion.position = 3.0;
  current_state.frontLeftWheelSpinningMotion.velocity = 4.0;
  current_state.frontLeftWheelSpinningMotion.torque = 5.0;
  current_state.frontRightWheelSpinningMotion.position = 6.0;
  current_state.frontRightWheelSpinningMotion.velocity = 7.0;
  current_state.frontRightWheelSpinningMotion.torque = 8.0;
  current_state.rearLeftWheelSpinningMotion.position = 9.0;
  current_state.rearLeftWheelSpinningMotion.velocity = 10.0;
  current_state.rearLeftWheelSpinningMotion.torque = 11.0;
  current_state.rearRightWheelSpinningMotion.position = 12.0;
  current_state.rearRightWheelSpinningMotion.velocity = 13.0;
  current_state.rearRightWheelSpinningMotion.torque = 14.0;

  interface->set_state(current_state);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 14; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2FWS4WD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 6; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::HardwareCommand2FWS4WD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSpinningSetPoint, 3.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSpinningSetPoint, 4.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSpinningSetPoint, 5.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSpinningSetPoint, 6.0);
}
