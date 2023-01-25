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
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"

class TestHarwareInterface2TTD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface2TTD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface2TTD.urdf";
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
    interface = std::make_unique<romea::HardwareInterface2TTD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::HardwareInterface2TTD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2TTD, checkJointNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  EXPECT_STREQ(state_interfaces[0].get_name().c_str(), "robot_joint1");
  EXPECT_STREQ(state_interfaces[3].get_name().c_str(), "robot_joint2");
  EXPECT_STREQ(state_interfaces[6].get_name().c_str(), "robot_joint3");
  EXPECT_STREQ(state_interfaces[9].get_name().c_str(), "robot_joint4");
  EXPECT_STREQ(state_interfaces[12].get_name().c_str(), "robot_joint5");
  EXPECT_STREQ(state_interfaces[15].get_name().c_str(), "robot_joint6");
  EXPECT_STREQ(state_interfaces[18].get_name().c_str(), "robot_joint7");
  EXPECT_STREQ(state_interfaces[21].get_name().c_str(), "robot_joint8");

  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_joint_name().c_str(),"robot_joint2");

  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.feedback.velocity.get_joint_name().c_str(),"robot_joint1");
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.feedback.velocity.get_joint_name().c_str(),"robot_joint2");

  //  EXPECT_STREQ(interface->left_idler_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint3");
  //  EXPECT_STREQ(interface->right_idler_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint4");
  //  EXPECT_STREQ(interface->front_left_roller_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint5");
  //  EXPECT_STREQ(interface->front_right_roller_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint6");
  //  EXPECT_STREQ(interface->rear_left_roller_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint7");
  //  EXPECT_STREQ(interface->rear_right_roller_wheel_spinning_joint_feedback.velocity.get_joint_name().c_str(),"robot_joint8");
}

TEST_F(TestHarwareInterface2TTD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/velocity");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/velocity");
  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_VELOCITY);
}

TEST_F(TestHarwareInterface2TTD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "robot_joint1/effort");
  EXPECT_STREQ(command_interfaces[1].get_full_name().c_str(), "robot_joint2/effort");
  //  EXPECT_STREQ(interface->left_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
  //  EXPECT_STREQ(interface->right_sprocket_wheel_spinning_joint.command.get_interface_type().c_str(),hardware_interface::HW_IF_EFFORT);
}

TEST_F(TestHarwareInterface2TTD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::HardwareState2TD current_state;
  current_state.leftSprocketWheelSpinningMotion.position = 1.0;
  current_state.leftSprocketWheelSpinningMotion.velocity = 2.0;
  current_state.leftSprocketWheelSpinningMotion.torque = 3.0;
  current_state.rightSprocketWheelSpinningMotion.position = 4.0;
  current_state.rightSprocketWheelSpinningMotion.velocity = 5.0;
  current_state.rightSprocketWheelSpinningMotion.torque = 6.0;
  romea::RotationalMotionState leftIdlerWheelSpinningMotion;
  leftIdlerWheelSpinningMotion.position = 7.0;
  leftIdlerWheelSpinningMotion.velocity = 8.0;
  leftIdlerWheelSpinningMotion.torque = 9.0;
  romea::RotationalMotionState rightIdlerWheelSpinningMotion;
  rightIdlerWheelSpinningMotion.position = 10.0;
  rightIdlerWheelSpinningMotion.velocity = 11.0;
  rightIdlerWheelSpinningMotion.torque = 12.0;
  romea::RotationalMotionState frontLeftRollerWheelSpinningMotion;
  frontLeftRollerWheelSpinningMotion.position = 13.0;
  frontLeftRollerWheelSpinningMotion.velocity = 14.0;
  frontLeftRollerWheelSpinningMotion.torque = 15.0;
  romea::RotationalMotionState frontRightRollerWheelSpinningMotion;
  frontRightRollerWheelSpinningMotion.position = 16.0;
  frontRightRollerWheelSpinningMotion.velocity = 17.0;
  frontRightRollerWheelSpinningMotion.torque = 18.0;
  romea::RotationalMotionState rearLeftRollerWheelSpinningMotion;
  rearLeftRollerWheelSpinningMotion.position = 19.0;
  rearLeftRollerWheelSpinningMotion.velocity = 20.0;
  rearLeftRollerWheelSpinningMotion.torque = 21.0;
  romea::RotationalMotionState rearRightRollerWheelSpinningMotion;
  rearRightRollerWheelSpinningMotion.position = 22.0;
  rearRightRollerWheelSpinningMotion.velocity = 23.0;
  rearRightRollerWheelSpinningMotion.torque = 24.0;

  interface->set_state(
    current_state,
    leftIdlerWheelSpinningMotion,
    rightIdlerWheelSpinningMotion,
    frontLeftRollerWheelSpinningMotion,
    frontRightRollerWheelSpinningMotion,
    rearLeftRollerWheelSpinningMotion,
    rearRightRollerWheelSpinningMotion);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 24; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2TTD, checkGetCurrentCommand)
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
