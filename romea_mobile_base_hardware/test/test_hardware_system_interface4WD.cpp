// Copyright 2022 INRAE, French National Research Institute for Agriculture,
// Food and Environment
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

// gtest
#include "gtest/gtest.h"

// ros
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_interface4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface_base.hpp"
#include "test_utils.hpp"

class TestHardwareSystemInterface4WD : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_infos = parse_hardware_info("test_hardware_system_interface4WD.xacro");
    ASSERT_EQ(hardware_infos.size(), 1u);
  }

  std::vector<hardware_interface::HardwareInfo> hardware_infos;
};

TEST_F(
  TestHardwareSystemInterface4WD,
  checkFactoryCreatesInterfaceFromPrefixedParameters)
{
  auto hardware_interface =
    romea::ros2::make_hardware_interface(hardware_infos[0], "4WD", "mobile_base");

  auto & interface = dynamic_cast<romea::ros2::HardwareInterface4WD &>(*hardware_interface);

  auto joint_names = interface.get_joint_names();
  ASSERT_EQ(joint_names.size(), 4u);
  EXPECT_EQ(joint_names[0], "front_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[1], "front_right_wheel_spinning_joint");
  EXPECT_EQ(joint_names[2], "rear_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[3], "rear_right_wheel_spinning_joint");
}

TEST_F(
  TestHardwareSystemInterface4WD,
  checkSystemInterfaceLoadsDynamicInterface)
{
  TestableHardwareSystemInterface system_interface;

  EXPECT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WD>("mobile_base");

  auto joint_names = interface.get_joint_names();
  ASSERT_EQ(joint_names.size(), 4u);
  EXPECT_EQ(joint_names[0], "front_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[1], "front_right_wheel_spinning_joint");
  EXPECT_EQ(joint_names[2], "rear_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[3], "rear_right_wheel_spinning_joint");
}

TEST_F(
  TestHardwareSystemInterface4WD,
  checkSystemInterfaceExportsStateAndCommandInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 12u);
  expect_interface_name(state_interfaces[0], "front_left_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[3], "front_right_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[6], "rear_left_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[9], "rear_right_wheel_spinning_joint/position");

  auto command_interfaces = system_interface.export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 4u);
  expect_interface_name(command_interfaces[0], "front_left_wheel_spinning_joint/velocity");
  expect_interface_name(command_interfaces[1], "front_right_wheel_spinning_joint/velocity");
  expect_interface_name(command_interfaces[2], "rear_left_wheel_spinning_joint/velocity");
  expect_interface_name(command_interfaces[3], "rear_right_wheel_spinning_joint/velocity");
}

TEST_F(
  TestHardwareSystemInterface4WD,
  checkCommandValuesArePackedIntoJointState)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto command_interfaces = system_interface.export_command_interfaces();
  for (size_t n = 0; n < 4; ++n) {
    ASSERT_TRUE(command_interfaces[n].set_value(n + 1.0));
  }

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WD>("mobile_base");
  auto command = interface.get_joint_state_command();

  ASSERT_EQ(command.name.size(), 4u);
  EXPECT_EQ(command.name[0], "front_left_wheel_spinning_joint");
  EXPECT_EQ(command.name[1], "front_right_wheel_spinning_joint");
  EXPECT_EQ(command.name[2], "rear_left_wheel_spinning_joint");
  EXPECT_EQ(command.name[3], "rear_right_wheel_spinning_joint");

  EXPECT_DOUBLE_EQ(command.velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(command.velocity[1], 2.0);
  EXPECT_DOUBLE_EQ(command.velocity[2], 3.0);
  EXPECT_DOUBLE_EQ(command.velocity[3], 4.0);
}

TEST_F(
  TestHardwareSystemInterface4WD,
  checkJointStateFeedbackValuesAreExportedIntoStateInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WD>("mobile_base");

  auto feedback = romea::ros2::make_joint_state_msg(4);
  feedback.name[0] = "front_left_wheel_spinning_joint";
  feedback.name[1] = "front_right_wheel_spinning_joint";
  feedback.name[2] = "rear_left_wheel_spinning_joint";
  feedback.name[3] = "rear_right_wheel_spinning_joint";
  feedback.position[0] = 1.0;
  feedback.velocity[0] = 2.0;
  feedback.effort[0] = 3.0;
  feedback.position[1] = 4.0;
  feedback.velocity[1] = 5.0;
  feedback.effort[1] = 6.0;
  feedback.position[2] = 7.0;
  feedback.velocity[2] = 8.0;
  feedback.effort[2] = 9.0;
  feedback.position[3] = 10.0;
  feedback.velocity[3] = 11.0;
  feedback.effort[3] = 12.0;

  interface.set_feedback(feedback);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 12u);
  for (size_t n = 0; n < 12; ++n) {
    EXPECT_DOUBLE_EQ(state_interfaces[n].get_value(), n + 1.0);
  }
}

TEST_F(
  TestHardwareSystemInterface4WD,
  checkCommandValuesAreAvailableAsCoreHardwareCommand)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto command_interfaces = system_interface.export_command_interfaces();
  for (size_t n = 0; n < 4; ++n) {
    ASSERT_TRUE(command_interfaces[n].set_value(n + 1.0));
  }

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WD>("mobile_base");
  auto command = interface.get_hardware_command();

  EXPECT_DOUBLE_EQ(command.frontLeftWheelSpinningSetPoint, 1.0);
  EXPECT_DOUBLE_EQ(command.frontRightWheelSpinningSetPoint, 2.0);
  EXPECT_DOUBLE_EQ(command.rearLeftWheelSpinningSetPoint, 3.0);
  EXPECT_DOUBLE_EQ(command.rearRightWheelSpinningSetPoint, 4.0);
}

TEST_F(
  TestHardwareSystemInterface4WD,
  checkCoreHardwareStateValuesAreExportedIntoStateInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WD>("mobile_base");

  romea::core::HardwareState4WD state;
  state.frontLeftWheelSpinningMotion.position = 1.0;
  state.frontLeftWheelSpinningMotion.velocity = 2.0;
  state.frontLeftWheelSpinningMotion.torque = 3.0;
  state.frontRightWheelSpinningMotion.position = 4.0;
  state.frontRightWheelSpinningMotion.velocity = 5.0;
  state.frontRightWheelSpinningMotion.torque = 6.0;
  state.rearLeftWheelSpinningMotion.position = 7.0;
  state.rearLeftWheelSpinningMotion.velocity = 8.0;
  state.rearLeftWheelSpinningMotion.torque = 9.0;
  state.rearRightWheelSpinningMotion.position = 10.0;
  state.rearRightWheelSpinningMotion.velocity = 11.0;
  state.rearRightWheelSpinningMotion.torque = 12.0;

  interface.set_feedback(state);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 12u);
  for (size_t n = 0; n < 12; ++n) {
    EXPECT_DOUBLE_EQ(state_interfaces[n].get_value(), n + 1.0);
  }
}
