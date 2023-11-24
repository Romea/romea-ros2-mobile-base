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

TEST_F(TestHarwareInterface2FWS4WD, checkSetCurrentState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::core::HardwareState2FWS4WD current_state;
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

  romea::core::HardwareCommand2FWS4WD current_command = interface->get_command();

  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(current_command.frontLeftWheelSpinningSetPoint, 3.0);
  EXPECT_DOUBLE_EQ(current_command.frontRightWheelSpinningSetPoint, 4.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSpinningSetPoint, 5.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSpinningSetPoint, 6.0);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
