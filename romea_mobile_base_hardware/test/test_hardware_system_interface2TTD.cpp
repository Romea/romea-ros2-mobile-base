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

#include "gtest/gtest.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"
#include "romea_mobile_base_hardware/hardware_interface_base.hpp"
#include "test_utils.hpp"

class TestHardwareSystemInterface2TTD : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_infos = parse_hardware_info("test_hardware_system_interface2TTD.xacro");
    ASSERT_EQ(hardware_infos.size(), 1u);
  }

  std::vector<hardware_interface::HardwareInfo> hardware_infos;
};

TEST_F(
  TestHardwareSystemInterface2TTD,
  checkFactoryCreatesInterfaceFromPrefixedParameters)
{
  auto hardware_interface =
    romea::ros2::make_hardware_interface(hardware_infos[0], "2TTD", "mobile_base");

  auto & interface = dynamic_cast<romea::ros2::HardwareInterface2TTD &>(*hardware_interface);

  auto joint_names = interface.get_joint_names();
  ASSERT_EQ(joint_names.size(), 2u);
  EXPECT_EQ(joint_names[0], "left_sprocket_wheel_spinning_joint");
  EXPECT_EQ(joint_names[1], "right_sprocket_wheel_spinning_joint");
}

TEST_F(
  TestHardwareSystemInterface2TTD,
  checkSystemInterfaceLoadsDynamicInterface)
{
  TestableHardwareSystemInterface system_interface;

  EXPECT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface2TTD>("mobile_base");

  auto joint_names = interface.get_joint_names();
  ASSERT_EQ(joint_names.size(), 2u);
  EXPECT_EQ(joint_names[0], "left_sprocket_wheel_spinning_joint");
  EXPECT_EQ(joint_names[1], "right_sprocket_wheel_spinning_joint");
}

TEST_F(
  TestHardwareSystemInterface2TTD,
  checkSystemInterfaceExportsStateAndCommandInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 24u);
  expect_interface_name(state_interfaces[0], "left_sprocket_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[3], "right_sprocket_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[6], "left_idler_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[9], "right_idler_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[12], "front_left_roller_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[15], "front_right_roller_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[18], "rear_left_roller_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[21], "rear_right_roller_wheel_spinning_joint/position");

  auto command_interfaces = system_interface.export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 2u);
  expect_interface_name(command_interfaces[0], "left_sprocket_wheel_spinning_joint/velocity");
  expect_interface_name(command_interfaces[1], "right_sprocket_wheel_spinning_joint/velocity");
}

TEST_F(
  TestHardwareSystemInterface2TTD,
  checkCommandValuesArePackedIntoJointState)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto command_interfaces = system_interface.export_command_interfaces();
  ASSERT_TRUE(command_interfaces[0].set_value(1.0));
  ASSERT_TRUE(command_interfaces[1].set_value(2.0));

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface2TTD>("mobile_base");
  auto command = interface.get_joint_state_command();

  ASSERT_EQ(command.name.size(), 2u);
  EXPECT_EQ(command.name[0], "left_sprocket_wheel_spinning_joint");
  EXPECT_EQ(command.name[1], "right_sprocket_wheel_spinning_joint");
  EXPECT_DOUBLE_EQ(command.velocity[0], 1.0);
  EXPECT_DOUBLE_EQ(command.velocity[1], 2.0);
}

TEST_F(
  TestHardwareSystemInterface2TTD,
  checkJointStateFeedbackValuesAreExportedIntoStateInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface2TTD>("mobile_base");

  auto feedback = romea::ros2::make_joint_state_msg(2);
  feedback.name[0] = "left_sprocket_wheel_spinning_joint";
  feedback.name[1] = "right_sprocket_wheel_spinning_joint";
  feedback.position[0] = 1.0;
  feedback.velocity[0] = 2.0;
  feedback.effort[0] = 3.0;
  feedback.position[1] = 4.0;
  feedback.velocity[1] = 5.0;
  feedback.effort[1] = 6.0;

  interface.set_feedback(feedback);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 24u);
  for (size_t n = 0; n < 6; ++n) {
    EXPECT_DOUBLE_EQ(state_interfaces[n].get_value(), n + 1.0);
  }
}

TEST_F(
  TestHardwareSystemInterface2TTD,
  checkCommandValuesAreAvailableAsCoreHardwareCommand)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto command_interfaces = system_interface.export_command_interfaces();
  ASSERT_TRUE(command_interfaces[0].set_value(1.0));
  ASSERT_TRUE(command_interfaces[1].set_value(2.0));

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface2TTD>("mobile_base");
  auto command = interface.get_hardware_command();

  EXPECT_DOUBLE_EQ(command.leftSprocketWheelSpinningSetPoint, 1.0);
  EXPECT_DOUBLE_EQ(command.rightSprocketWheelSpinningSetPoint, 2.0);
}

TEST_F(
  TestHardwareSystemInterface2TTD,
  checkCoreHardwareStateValuesAreExportedIntoStateInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface2TTD>("mobile_base");

  romea::core::HardwareState2TD state;
  state.leftSprocketWheelSpinningMotion.position = 1.0;
  state.leftSprocketWheelSpinningMotion.velocity = 2.0;
  state.leftSprocketWheelSpinningMotion.torque = 3.0;
  state.rightSprocketWheelSpinningMotion.position = 4.0;
  state.rightSprocketWheelSpinningMotion.velocity = 5.0;
  state.rightSprocketWheelSpinningMotion.torque = 6.0;

  interface.set_feedback(state);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 24u);
  for (size_t n = 0; n < 6; ++n) {
    EXPECT_DOUBLE_EQ(state_interfaces[n].get_value(), n + 1.0);
  }
}
