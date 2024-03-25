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

// romea
#include "../test/test_helper.h"
#include "../test/test_utils.hpp"
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
    interface = std::make_unique<romea::ros2::HardwareInterface2FWS4WD>(
      info[0],
      command_interface_type);
  }

  std::unique_ptr<romea::ros2::HardwareInterface2FWS4WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2FWS4WD, checkJointNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  check_interface_name(state_interfaces[0], "robot_joint1/position");
  check_interface_name(state_interfaces[1], "robot_joint2/position");
  check_interface_name(state_interfaces[2], "robot_joint3/position");
  check_interface_name(state_interfaces[5], "robot_joint4/position");
  check_interface_name(state_interfaces[8], "robot_joint5/position");
  check_interface_name(state_interfaces[11], "robot_joint6/position");
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

TEST_F(TestHarwareInterface2FWS4WD, checkSetFeedback)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::core::HardwareState2FWS4WD feeback;
  feeback.frontLeftWheelSteeringAngle = 1.0;
  feeback.frontRightWheelSteeringAngle = 2.0;
  feeback.frontLeftWheelSpinningMotion.position = 3.0;
  feeback.frontLeftWheelSpinningMotion.velocity = 4.0;
  feeback.frontLeftWheelSpinningMotion.torque = 5.0;
  feeback.frontRightWheelSpinningMotion.position = 6.0;
  feeback.frontRightWheelSpinningMotion.velocity = 7.0;
  feeback.frontRightWheelSpinningMotion.torque = 8.0;
  feeback.rearLeftWheelSpinningMotion.position = 9.0;
  feeback.rearLeftWheelSpinningMotion.velocity = 10.0;
  feeback.rearLeftWheelSpinningMotion.torque = 11.0;
  feeback.rearRightWheelSpinningMotion.position = 12.0;
  feeback.rearRightWheelSpinningMotion.velocity = 13.0;
  feeback.rearRightWheelSpinningMotion.torque = 14.0;

  interface->set_feedback(feeback);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 14; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2FWS4WD, checkSetFeedbackUsingJointStates)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto feedback = romea::ros2::make_joint_state_msg(6);
  feedback.name[0] = "robot_joint1";
  feedback.name[1] = "robot_joint2";
  feedback.name[2] = "robot_joint3";
  feedback.name[3] = "robot_joint4";
  feedback.name[4] = "robot_joint5";
  feedback.name[5] = "robot_joint6";
  feedback.position[0] = 1.0;
  feedback.position[1] = 2.0;
  feedback.position[2] = 3.0;
  feedback.velocity[2] = 4.0;
  feedback.effort[2] = 5.0;
  feedback.position[3] = 6.0;
  feedback.velocity[3] = 7.0;
  feedback.effort[3] = 8.0;
  feedback.position[4] = 9.0;
  feedback.velocity[4] = 10.0;
  feedback.effort[4] = 11.0;
  feedback.position[5] = 12.0;
  feedback.velocity[5] = 13.0;
  feedback.effort[5] = 14.0;

  interface->set_feedback(feedback);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 14; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2FWS4WD, checkGetCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 6; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  auto command = interface->get_hardware_command();

  EXPECT_DOUBLE_EQ(command.frontLeftWheelSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(command.frontRightWheelSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(command.frontLeftWheelSpinningSetPoint, 3.0);
  EXPECT_DOUBLE_EQ(command.frontRightWheelSpinningSetPoint, 4.0);
  EXPECT_DOUBLE_EQ(command.rearLeftWheelSpinningSetPoint, 5.0);
  EXPECT_DOUBLE_EQ(command.rearRightWheelSpinningSetPoint, 6.0);
}

TEST_F(TestHarwareInterface2FWS4WD, checkGetCommandUsingJointState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 6; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  auto command = interface->get_joint_state_command();
  EXPECT_STREQ(command.name[0].c_str(), "robot_joint1");
  EXPECT_STREQ(command.name[1].c_str(), "robot_joint2");
  EXPECT_STREQ(command.name[2].c_str(), "robot_joint3");
  EXPECT_STREQ(command.name[3].c_str(), "robot_joint4");
  EXPECT_STREQ(command.name[4].c_str(), "robot_joint5");
  EXPECT_STREQ(command.name[5].c_str(), "robot_joint6");

  EXPECT_DOUBLE_EQ(command.position[0], 1.0);
  EXPECT_DOUBLE_EQ(command.position[1], 2.0);
  EXPECT_DOUBLE_EQ(command.velocity[2], 3.0);
  EXPECT_DOUBLE_EQ(command.velocity[3], 4.0);
  EXPECT_DOUBLE_EQ(command.velocity[4], 5.0);
  EXPECT_DOUBLE_EQ(command.velocity[5], 6.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
