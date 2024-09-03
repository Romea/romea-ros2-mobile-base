// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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
#include "../test/test_utils.hpp"
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
    interface = std::make_unique<romea::ros2::HardwareInterface4WS4WD>(
      info[0],
      command_interface_type);
  }

  std::unique_ptr<romea::ros2::HardwareInterface4WS4WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface4WS4WD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  check_interface_name(state_interfaces[0], "robot_joint1/position");
  check_interface_name(state_interfaces[1], "robot_joint2/position");
  check_interface_name(state_interfaces[2], "robot_joint3/position");
  check_interface_name(state_interfaces[3], "robot_joint4/position");
  check_interface_name(state_interfaces[4], "robot_joint5/position");
  check_interface_name(state_interfaces[7], "robot_joint6/position");
  check_interface_name(state_interfaces[10], "robot_joint7/position");
  check_interface_name(state_interfaces[13], "robot_joint8/position");
}


TEST_F(TestHarwareInterface4WS4WD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/position");
  check_interface_name(command_interfaces[1], "robot_joint2/position");
  check_interface_name(command_interfaces[2], "robot_joint3/position");
  check_interface_name(command_interfaces[3], "robot_joint4/position");
  check_interface_name(command_interfaces[4], "robot_joint5/velocity");
  check_interface_name(command_interfaces[5], "robot_joint6/velocity");
  check_interface_name(command_interfaces[6], "robot_joint7/velocity");
  check_interface_name(command_interfaces[7], "robot_joint8/velocity");
}

TEST_F(TestHarwareInterface4WS4WD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/position");
  check_interface_name(command_interfaces[1], "robot_joint2/position");
  check_interface_name(command_interfaces[2], "robot_joint3/position");
  check_interface_name(command_interfaces[3], "robot_joint4/position");
  check_interface_name(command_interfaces[4], "robot_joint5/effort");
  check_interface_name(command_interfaces[5], "robot_joint6/effort");
  check_interface_name(command_interfaces[6], "robot_joint7/effort");
  check_interface_name(command_interfaces[7], "robot_joint8/effort");
}


TEST_F(TestHarwareInterface4WS4WD, checkSetFeedback)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::core::HardwareState4WS4WD feedback;
  feedback.frontLeftWheelSteeringAngle = 1.0;
  feedback.frontRightWheelSteeringAngle = 2.0;
  feedback.rearLeftWheelSteeringAngle = 3.0;
  feedback.rearRightWheelSteeringAngle = 4.0;
  feedback.frontLeftWheelSpinningMotion.position = 5.0;
  feedback.frontLeftWheelSpinningMotion.velocity = 6.0;
  feedback.frontLeftWheelSpinningMotion.torque = 7.0;
  feedback.frontRightWheelSpinningMotion.position = 8.0;
  feedback.frontRightWheelSpinningMotion.velocity = 9.0;
  feedback.frontRightWheelSpinningMotion.torque = 10.0;
  feedback.rearLeftWheelSpinningMotion.position = 11.0;
  feedback.rearLeftWheelSpinningMotion.velocity = 12.0;
  feedback.rearLeftWheelSpinningMotion.torque = 13.0;
  feedback.rearRightWheelSpinningMotion.position = 14.0;
  feedback.rearRightWheelSpinningMotion.velocity = 15.0;
  feedback.rearRightWheelSpinningMotion.torque = 16.0;

  interface->set_feedback(feedback);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 16; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}


TEST_F(TestHarwareInterface4WS4WD, checkSetFeedbackUsingJointStates)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto feedback = romea::ros2::make_joint_state_msg(8);
  feedback.name[0] = "robot_joint1";
  feedback.name[1] = "robot_joint2";
  feedback.name[2] = "robot_joint3";
  feedback.name[3] = "robot_joint4";
  feedback.name[4] = "robot_joint5";
  feedback.name[5] = "robot_joint6";
  feedback.name[6] = "robot_joint7";
  feedback.name[7] = "robot_joint8";
  feedback.position[0] = 1.0;
  feedback.position[1] = 2.0;
  feedback.position[2] = 3.0;
  feedback.position[3] = 4.0;
  feedback.position[4] = 5.0;
  feedback.velocity[4] = 6.0;
  feedback.effort[4] = 7.0;
  feedback.position[5] = 8.0;
  feedback.velocity[5] = 9.0;
  feedback.effort[5] = 10.0;
  feedback.position[6] = 11.0;
  feedback.velocity[6] = 12.0;
  feedback.effort[6] = 13.0;
  feedback.position[7] = 14.0;
  feedback.velocity[7] = 15.0;
  feedback.effort[7] = 16.0;

  interface->set_feedback(feedback);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 16; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}


TEST_F(TestHarwareInterface4WS4WD, checkGetCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 8; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  auto command = interface->get_hardware_command();

  EXPECT_DOUBLE_EQ(command.frontLeftWheelSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(command.frontRightWheelSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(command.rearLeftWheelSteeringAngle, 3.0);
  EXPECT_DOUBLE_EQ(command.rearRightWheelSteeringAngle, 4.0);
  EXPECT_DOUBLE_EQ(command.frontLeftWheelSpinningSetPoint, 5.0);
  EXPECT_DOUBLE_EQ(command.frontRightWheelSpinningSetPoint, 6.0);
  EXPECT_DOUBLE_EQ(command.rearLeftWheelSpinningSetPoint, 7.0);
  EXPECT_DOUBLE_EQ(command.rearRightWheelSpinningSetPoint, 8.0);
}

TEST_F(TestHarwareInterface4WS4WD, checkGetCommandUsingJointState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 8; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  auto command = interface->get_joint_state_command();
  EXPECT_EQ(command.name.size(), 8u);
  EXPECT_STREQ(command.name[0].c_str(), "robot_joint1");
  EXPECT_STREQ(command.name[1].c_str(), "robot_joint2");
  EXPECT_STREQ(command.name[2].c_str(), "robot_joint3");
  EXPECT_STREQ(command.name[3].c_str(), "robot_joint4");
  EXPECT_STREQ(command.name[4].c_str(), "robot_joint5");
  EXPECT_STREQ(command.name[5].c_str(), "robot_joint6");
  EXPECT_STREQ(command.name[6].c_str(), "robot_joint7");
  EXPECT_STREQ(command.name[7].c_str(), "robot_joint8");

  EXPECT_DOUBLE_EQ(command.position[0], 1.0);
  EXPECT_DOUBLE_EQ(command.position[1], 2.0);
  EXPECT_DOUBLE_EQ(command.position[2], 3.0);
  EXPECT_DOUBLE_EQ(command.position[3], 4.0);
  EXPECT_DOUBLE_EQ(command.velocity[4], 5.0);
  EXPECT_DOUBLE_EQ(command.velocity[5], 6.0);
  EXPECT_DOUBLE_EQ(command.velocity[6], 7.0);
  EXPECT_DOUBLE_EQ(command.velocity[7], 8.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
