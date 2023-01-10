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

// local
#include "../test/test_helper.h"
#include "romea_mobile_base_hardware/hardware_interface1FAS4WD.hpp"

class TestHarwareInterface1FAS4WD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface1FAS4WD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface1FAS4WD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface1FAS4WD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface1FAS4WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface1FAS4WD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(), "robot_joint1");
  EXPECT_STREQ(state_interfaces[1].get_name().c_str(), "robot_joint2");
  EXPECT_STREQ(state_interfaces[4].get_name().c_str(), "robot_joint3");
  EXPECT_STREQ(state_interfaces[7].get_name().c_str(), "robot_joint4");
  EXPECT_STREQ(state_interfaces[10].get_name().c_str(), "robot_joint5");
  EXPECT_STREQ(state_interfaces[13].get_name().c_str(), "robot_joint6");
  EXPECT_STREQ(state_interfaces[14].get_name().c_str(), "robot_joint7");
}

TEST_F(TestHarwareInterface1FAS4WD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/velocity");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(), "robot_joint3/velocity");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(), "robot_joint4/velocity");
  EXPECT_STREQ(command_interfaces[4].get_full_name().c_str(), "robot_joint5/velocity");
}

TEST_F(TestHarwareInterface1FAS4WD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/position");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/effort");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(), "robot_joint3/effort");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(), "robot_joint4/effort");
  EXPECT_STREQ(command_interfaces[4].get_full_name().c_str(), "robot_joint5/effort");
}

TEST_F(TestHarwareInterface1FAS4WD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState1FAS4WD current_state;
  current_state.frontAxleSteeringAngle = 1.0;
  current_state.frontLeftWheelSpinningMotion.position = 2.0;
  current_state.frontLeftWheelSpinningMotion.velocity = 3.0;
  current_state.frontLeftWheelSpinningMotion.torque = 4.0;
  current_state.frontRightWheelSpinningMotion.position = 5.0;
  current_state.frontRightWheelSpinningMotion.velocity = 6.0;
  current_state.frontRightWheelSpinningMotion.torque = 7.0;
  current_state.rearLeftWheelSpinningMotion.position = 8.0;
  current_state.rearLeftWheelSpinningMotion.velocity = 9.0;
  current_state.rearLeftWheelSpinningMotion.torque = 10.0;
  current_state.rearRightWheelSpinningMotion.position = 11.0;
  current_state.rearRightWheelSpinningMotion.velocity = 12.0;
  current_state.rearRightWheelSpinningMotion.torque = 13.0;

  romea::SteeringAngleState front_left_wheel_steering_angle = 14.0;
  romea::SteeringAngleState front_right_wheel_steering_angle = 15.0;

  interface->set_state(
    current_state,
    front_left_wheel_steering_angle,
    front_right_wheel_steering_angle);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 15; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface1FAS4WD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 5; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::HardwareCommand1FAS4WD current_command = interface->get_command();
  EXPECT_DOUBLE_EQ(current_command.frontAxleSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSpinningSetPoint, 2.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSpinningSetPoint, 3.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSpinningSetPoint, 4.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSpinningSetPoint, 5.0);
}
