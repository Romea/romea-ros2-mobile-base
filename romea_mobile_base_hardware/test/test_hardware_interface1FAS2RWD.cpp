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
#include "romea_mobile_base_hardware/hardware_interface1FAS2RWD.hpp"

class TestHarwareInterface1FAS2RWD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_interface1FAS2RWD.xacro";
    std::string urdf_file = "/tmp/test_hardware_interface1FAS2RWD.urdf";
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
    interface = std::make_unique<romea::ros2::HardwareInterface1FAS2RWD>(
      info[0],
      command_interface_type);
  }

  std::unique_ptr<romea::ros2::HardwareInterface1FAS2RWD> interface;
  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHarwareInterface1FAS2RWD, checkStateInterfaceNames)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto state_interfaces = interface->export_state_interfaces();
  check_interface_name(state_interfaces[0], "robot_joint1/position");
  check_interface_name(state_interfaces[1], "robot_joint2/position");
  check_interface_name(state_interfaces[4], "robot_joint3/position");
  check_interface_name(state_interfaces[7], "robot_joint4/position");
  check_interface_name(state_interfaces[8], "robot_joint5/position");
  check_interface_name(state_interfaces[9], "robot_joint6/position");
  check_interface_name(state_interfaces[12], "robot_joint7/position");
}

TEST_F(TestHarwareInterface1FAS2RWD, checkCommandInterfaceTypeWhenVelocityControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/position");
  check_interface_name(command_interfaces[1], "robot_joint2/velocity");
  check_interface_name(command_interfaces[2], "robot_joint3/velocity");
}

TEST_F(TestHarwareInterface1FAS2RWD, DISABLED_checkCommandInterfaceTypeWhenEffortControlIsUsed)
{
  MakeInterface(hardware_interface::HW_IF_EFFORT);
  auto command_interfaces = interface->export_command_interfaces();
  check_interface_name(command_interfaces[0], "robot_joint1/position");
  check_interface_name(command_interfaces[1], "robot_joint2/effort");
  check_interface_name(command_interfaces[2], "robot_joint3/effort");
}

// TEST_F(TestHarwareInterface1FAS2RWD, checkSetCurrentState)
// {
//   MakeInterface(hardware_interface::HW_IF_VELOCITY);

//   romea::core::HardwareState1FAS2RWD current_state;
//   current_state.frontAxleSteeringAngle = 1.0;
//   current_state.rearLeftWheelSpinningMotion.position = 2.0;
//   current_state.rearLeftWheelSpinningMotion.velocity = 3.0;
//   current_state.rearLeftWheelSpinningMotion.torque = 4.0;
//   current_state.rearRightWheelSpinningMotion.position = 5.0;
//   current_state.rearRightWheelSpinningMotion.velocity = 6.0;
//   current_state.rearRightWheelSpinningMotion.torque = 7.0;

//   romea::core::SteeringAngleState front_left_wheel_steering_angle = 8.0;
//   romea::core::SteeringAngleState front_right_wheel_steering_angle = 9.0;
//   romea::core::RotationalMotionState front_left_wheel_spinning_set_point;
//   front_left_wheel_spinning_set_point.position = 10.0;
//   front_left_wheel_spinning_set_point.velocity = 11.0;
//   front_left_wheel_spinning_set_point.torque = 12.0;
//   romea::core::RotationalMotionState front_right_wheel_spinning_set_point;
//   front_right_wheel_spinning_set_point.position = 13.0;
//   front_right_wheel_spinning_set_point.velocity = 14.0;
//   front_right_wheel_spinning_set_point.torque = 15.0;

//   interface->set_state(
//     current_state,
//     front_left_wheel_steering_angle,
//     front_right_wheel_steering_angle,
//     front_left_wheel_spinning_set_point,
//     front_right_wheel_spinning_set_point);

//   auto state_interfaces = interface->export_state_interfaces();
//   for (size_t i = 0; i < 15; ++i) {
//     EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
//   }
// }

TEST_F(TestHarwareInterface1FAS2RWD, checkSetFeedback)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  romea::core::HardwareState1FAS2RWD current_state;
  current_state.frontAxleSteeringAngle = 1.0;
  current_state.rearLeftWheelSpinningMotion.position = 2.0;
  current_state.rearLeftWheelSpinningMotion.velocity = 3.0;
  current_state.rearLeftWheelSpinningMotion.torque = 4.0;
  current_state.rearRightWheelSpinningMotion.position = 5.0;
  current_state.rearRightWheelSpinningMotion.velocity = 6.0;
  current_state.rearRightWheelSpinningMotion.torque = 7.0;

  interface->set_feedback(current_state);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}

TEST_F(TestHarwareInterface1FAS2RWD, checkSetFeedbackUsingJointStates)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto feedback = romea::ros2::make_joint_state_msg(3);
  feedback.name[0] = "robot_joint1";
  feedback.name[1] = "robot_joint2";
  feedback.name[2] = "robot_joint3";
  feedback.position[0] = 1.0;
  feedback.velocity[0] = 0.0;
  feedback.effort[0] = 0.0;
  feedback.position[1] = 2.0;
  feedback.velocity[1] = 3.0;
  feedback.effort[1] = 4.0;
  feedback.position[2] = 5.0;
  feedback.velocity[2] = 6.0;
  feedback.effort[2] = 7.0;

  interface->set_feedback(feedback);

  auto state_interfaces = interface->export_state_interfaces();
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(state_interfaces[i].get_value(), i + 1.0);
  }
}


TEST_F(TestHarwareInterface1FAS2RWD, checkGetCommand)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 3; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  romea::core::HardwareCommand1FAS2RWD current_command = interface->get_hardware_command();
  EXPECT_DOUBLE_EQ(current_command.frontAxleSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(current_command.rearLeftWheelSpinningSetPoint, 2.0);
  EXPECT_DOUBLE_EQ(current_command.rearRightWheelSpinningSetPoint, 3.0);
}

TEST_F(TestHarwareInterface1FAS2RWD, checkGetCommandUsingJointState)
{
  MakeInterface(hardware_interface::HW_IF_VELOCITY);

  auto command_interfaces = interface->export_command_interfaces();
  for (size_t i = 0; i < 3; ++i) {
    command_interfaces[i].set_value(i + 1.0);
  }

  auto command = interface->get_joint_state_command();
  EXPECT_EQ(command.name.size(), 3u);
  EXPECT_STREQ(command.name[0].c_str(), "robot_joint1");
  EXPECT_STREQ(command.name[1].c_str(), "robot_joint2");
  EXPECT_STREQ(command.name[2].c_str(), "robot_joint3");

  EXPECT_DOUBLE_EQ(command.position[0], 1.0);
  EXPECT_DOUBLE_EQ(command.velocity[1], 2.0);
  EXPECT_DOUBLE_EQ(command.velocity[2], 3.0);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
