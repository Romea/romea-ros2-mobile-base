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
    interface =
      std::make_unique<romea::ros2::HardwareInterface2TTD>(info[0], command_interface_type);
  }

  std::unique_ptr<romea::ros2::HardwareInterface2TTD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface2TTD, checkJointNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  check_interface_name(state_interfaces[0], "robot_joint1/position");
  check_interface_name(state_interfaces[3], "robot_joint2/position");
  check_interface_name(state_interfaces[6], "robot_joint3/position");
  check_interface_name(state_interfaces[9], "robot_joint4/position");
  check_interface_name(state_interfaces[12], "robot_joint5/position");
  check_interface_name(state_interfaces[15], "robot_joint6/position");
  check_interface_name(state_interfaces[18], "robot_joint7/position");
  check_interface_name(state_interfaces[21], "robot_joint8/position");
}

TEST_F(TestHarwareInterface2TTD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/velocity");
  check_interface_name(command_interfaces[1], "robot_joint2/velocity");
}

TEST_F(TestHarwareInterface2TTD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/effort");
  check_interface_name(command_interfaces[1], "robot_joint2/effort");
}

// TEST_F(TestHarwareInterface2TTD, checkSetCurrentState)
// {
//   MakeInterface(hardware_interface::HW_IF_VELOCITY);

//   romea::core::HardwareState2TD current_state;
//   current_state.leftSprocketWheelSpinningMotion.position = 1.0;
//   current_state.leftSprocketWheelSpinningMotion.velocity = 2.0;
//   current_state.leftSprocketWheelSpinningMotion.torque = 3.0;
//   current_state.rightSprocketWheelSpinningMotion.position = 4.0;
//   current_state.rightSprocketWheelSpinningMotion.velocity = 5.0;
//   current_state.rightSprocketWheelSpinningMotion.torque = 6.0;
//   romea::core::RotationalMotionState leftIdlerWheelSpinningMotion;
//   leftIdlerWheelSpinningMotion.position = 7.0;
//   leftIdlerWheelSpinningMotion.velocity = 8.0;
//   leftIdlerWheelSpinningMotion.torque = 9.0;
//   romea::core::RotationalMotionState rightIdlerWheelSpinningMotion;
//   rightIdlerWheelSpinningMotion.position = 10.0;
//   rightIdlerWheelSpinningMotion.velocity = 11.0;
//   rightIdlerWheelSpinningMotion.torque = 12.0;
//   romea::core::RotationalMotionState frontLeftRollerWheelSpinningMotion;
//   frontLeftRollerWheelSpinningMotion.position = 13.0;
//   frontLeftRollerWheelSpinningMotion.velocity = 14.0;
//   frontLeftRollerWheelSpinningMotion.torque = 15.0;
//   romea::core::RotationalMotionState frontRightRollerWheelSpinningMotion;
//   frontRightRollerWheelSpinningMotion.position = 16.0;
//   frontRightRollerWheelSpinningMotion.velocity = 17.0;
//   frontRightRollerWheelSpinningMotion.torque = 18.0;
//   romea::core::RotationalMotionState rearLeftRollerWheelSpinningMotion;
//   rearLeftRollerWheelSpinningMotion.position = 19.0;
//   rearLeftRollerWheelSpinningMotion.velocity = 20.0;
//   rearLeftRollerWheelSpinningMotion.torque = 21.0;
//   romea::core::RotationalMotionState rearRightRollerWheelSpinningMotion;
//   rearRightRollerWheelSpinningMotion.position = 22.0;
//   rearRightRollerWheelSpinningMotion.velocity = 23.0;
//   rearRightRollerWheelSpinningMotion.torque = 24.0;

//   interface->set_state(
//     current_state,
//     leftIdlerWheelSpinningMotion,
//     rightIdlerWheelSpinningMotion,
//     frontLeftRollerWheelSpinningMotion,
//     frontRightRollerWheelSpinningMotion,
//     rearLeftRollerWheelSpinningMotion,
//     rearRightRollerWheelSpinningMotion);

//   auto state_interfaces = interface->export_state_interfaces();
//   for (size_t i = 0; i < 24; ++i) {
//     EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
//   }
// }

TEST_F(TestHarwareInterface2TTD, checkSetFeedback)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::core::HardwareState2TD current_state;
  current_state.leftSprocketWheelSpinningMotion.position = 1.0;
  current_state.leftSprocketWheelSpinningMotion.velocity = 2.0;
  current_state.leftSprocketWheelSpinningMotion.torque = 3.0;
  current_state.rightSprocketWheelSpinningMotion.position = 4.0;
  current_state.rightSprocketWheelSpinningMotion.velocity = 5.0;
  current_state.rightSprocketWheelSpinningMotion.torque = 6.0;

  interface->set_feedback(current_state);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface2TTD, checkGetCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 2; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::core::HardwareCommand2TD current_command = interface->get_hardware_command();

  EXPECT_DOUBLE_EQ(current_command.leftSprocketWheelSpinningSetPoint, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rightSprocketWheelSpinningSetPoint, 2.0);
}
