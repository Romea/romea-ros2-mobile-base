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
#include <memory>
#include <fstream>
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
    interface =
      std::make_unique<romea::ros2::HardwareInterface2WD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::ros2::HardwareInterface2WD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2WD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  check_interface_name(state_interfaces[0], "robot_joint1/position");
  check_interface_name(state_interfaces[3], "robot_joint2/position");
}

TEST_F(TestHarwareInterface2WD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/velocity");
  check_interface_name(command_interfaces[1], "robot_joint2/velocity");
}

TEST_F(TestHarwareInterface2WD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/effort");
  check_interface_name(command_interfaces[1], "robot_joint2/effort");
}

TEST_F(TestHarwareInterface2WD, checkSetFeedback)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::core::HardwareState2WD current_state;
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

TEST_F(TestHarwareInterface4WD, checkSetFeedbackUsingJointStates)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto feedback = romea::ros2::make_joint_state_msg(4);
  feedback.name[0] = "robot_joint1";
  feedback.name[1] = "robot_joint2";
  feedback.position[0] = 1.0;
  feedback.velocity[0] = 2.0;
  feedback.effort[0] = 3.0;
  feedback.position[1] = 4.0;
  feedback.velocity[1] = 5.0;
  feedback.effort[1] = 6.0;

  interface->set_feedback(feedback);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}


TEST_F(TestHarwareInterface2WD, checkGetCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 2; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::core::HardwareCommand2WD current_command = interface->get_command();
  EXPECT_DOUBLE_EQ(current_command.leftWheelSpinningSetPoint, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rightWheelSpinningSetPoint, 2.0);
}

TEST_F(TestHarwareInterface4WD, checkGetCommandUsingJointState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 2; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  auto command = interface->get_joint_state_command();
  EXPECT_STREQ(command.name[0].c_str(), "robot_joint1");
  EXPECT_STREQ(command.name[1].c_str(), "robot_joint2");

  EXPECT_DOUBLE_EQ(command.velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(command.velocity[1], 2.0);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
