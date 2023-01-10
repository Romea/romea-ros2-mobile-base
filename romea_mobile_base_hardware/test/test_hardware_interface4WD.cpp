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
#include "romea_mobile_base_hardware/hardware_interface4WD.hpp"

class TestHarwareInterface4WD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface4WD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface4WD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface4WD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface4WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface4WD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(), "robot_joint1");
  EXPECT_STREQ(state_interfaces[3].get_name().c_str(), "robot_joint2");
  EXPECT_STREQ(state_interfaces[6].get_name().c_str(), "robot_joint3");
  EXPECT_STREQ(state_interfaces[9].get_name().c_str(), "robot_joint4");

  //  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint2");
  //  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint3");
  //  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint4");
  //  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.feedback.position.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.feedback.position.get_joint_name().c_str(),"robot_joint2");
  //  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint.feedback.position.get_joint_name().c_str(),"robot_joint3");
  //  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint.feedback.position.get_joint_name().c_str(),"robot_joint4");
}

TEST_F(TestHarwareInterface4WD, checkCommandInterfaceTypeWhenVelocityCovntrolIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/velocity");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/velocity");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(), "robot_joint3/velocity");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(), "robot_joint4/velocity");

  //  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
}

TEST_F(TestHarwareInterface4WD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/effort");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/effort");
  EXPECT_STREQ(command_interfaces[2].get_full_name().c_str(), "robot_joint3/effort");
  EXPECT_STREQ(command_interfaces[3].get_full_name().c_str(), "robot_joint4/effort");

  //  EXPECT_STREQ(interface->front_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->front_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->rear_left_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->rear_right_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
}

TEST_F(TestHarwareInterface4WD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState4WD current_state;
  current_state.frontLeftWheelSpinningMotion.position = 1.0;
  current_state.frontLeftWheelSpinningMotion.velocity = 2.0;
  current_state.frontLeftWheelSpinningMotion.torque = 3.0;
  current_state.frontRightWheelSpinningMotion.position = 4.0;
  current_state.frontRightWheelSpinningMotion.velocity = 5.0;
  current_state.frontRightWheelSpinningMotion.torque = 6.0;
  current_state.rearLeftWheelSpinningMotion.position = 7.0;
  current_state.rearLeftWheelSpinningMotion.velocity = 8.0;
  current_state.rearLeftWheelSpinningMotion.torque = 9.0;
  current_state.rearRightWheelSpinningMotion.position = 10.0;
  current_state.rearRightWheelSpinningMotion.velocity = 11.0;
  current_state.rearRightWheelSpinningMotion.torque = 12.0;

  interface->set_state(current_state);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 12; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface4WD, checkGetCurrentCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::HardwareCommand4WD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSpinningSetPoint, 1.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSpinningSetPoint, 2.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSpinningSetPoint, 3.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSpinningSetPoint, 4.0);
}
