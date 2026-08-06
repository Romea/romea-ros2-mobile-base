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

#include "romea_mobile_base_hardware/hardware_interface4WS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface_base.hpp"
#include "test_utils.hpp"

class TestHardwareSystemInterface4WS4WD : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_infos = parse_hardware_info("test_hardware_system_interface4WS4WD.xacro");
    ASSERT_EQ(hardware_infos.size(), 1u);
  }

  std::vector<hardware_interface::HardwareInfo> hardware_infos;
};

TEST_F(
  TestHardwareSystemInterface4WS4WD,
  checkFactoryCreatesInterfaceFromPrefixedParameters)
{
  auto hardware_interface =
    romea::ros2::make_hardware_interface(hardware_infos[0], "4WS4WD", "mobile_base");

  auto & interface = dynamic_cast<romea::ros2::HardwareInterface4WS4WD &>(*hardware_interface);

  auto joint_names = interface.get_joint_names();
  ASSERT_EQ(joint_names.size(), 8u);
  EXPECT_EQ(joint_names[0], "front_left_wheel_steering_joint");
  EXPECT_EQ(joint_names[1], "front_right_wheel_steering_joint");
  EXPECT_EQ(joint_names[2], "rear_left_wheel_steering_joint");
  EXPECT_EQ(joint_names[3], "rear_right_wheel_steering_joint");
  EXPECT_EQ(joint_names[4], "front_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[5], "front_right_wheel_spinning_joint");
  EXPECT_EQ(joint_names[6], "rear_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[7], "rear_right_wheel_spinning_joint");
}

TEST_F(
  TestHardwareSystemInterface4WS4WD,
  checkSystemInterfaceLoadsDynamicInterface)
{
  TestableHardwareSystemInterface system_interface;

  EXPECT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WS4WD>("mobile_base");

  auto joint_names = interface.get_joint_names();
  ASSERT_EQ(joint_names.size(), 8u);
  EXPECT_EQ(joint_names[0], "front_left_wheel_steering_joint");
  EXPECT_EQ(joint_names[1], "front_right_wheel_steering_joint");
  EXPECT_EQ(joint_names[2], "rear_left_wheel_steering_joint");
  EXPECT_EQ(joint_names[3], "rear_right_wheel_steering_joint");
  EXPECT_EQ(joint_names[4], "front_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[5], "front_right_wheel_spinning_joint");
  EXPECT_EQ(joint_names[6], "rear_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[7], "rear_right_wheel_spinning_joint");
}

TEST_F(
  TestHardwareSystemInterface4WS4WD,
  checkSystemInterfaceExportsStateAndCommandInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 16u);
  expect_interface_name(state_interfaces[0], "front_left_wheel_steering_joint/position");
  expect_interface_name(state_interfaces[1], "front_right_wheel_steering_joint/position");
  expect_interface_name(state_interfaces[2], "rear_left_wheel_steering_joint/position");
  expect_interface_name(state_interfaces[3], "rear_right_wheel_steering_joint/position");
  expect_interface_name(state_interfaces[4], "front_left_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[7], "front_right_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[10], "rear_left_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[13], "rear_right_wheel_spinning_joint/position");

  auto command_interfaces = system_interface.export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 8u);
  expect_interface_name(command_interfaces[0], "front_left_wheel_steering_joint/position");
  expect_interface_name(command_interfaces[1], "front_right_wheel_steering_joint/position");
  expect_interface_name(command_interfaces[2], "rear_left_wheel_steering_joint/position");
  expect_interface_name(command_interfaces[3], "rear_right_wheel_steering_joint/position");
  expect_interface_name(command_interfaces[4], "front_left_wheel_spinning_joint/velocity");
  expect_interface_name(command_interfaces[5], "front_right_wheel_spinning_joint/velocity");
  expect_interface_name(command_interfaces[6], "rear_left_wheel_spinning_joint/velocity");
  expect_interface_name(command_interfaces[7], "rear_right_wheel_spinning_joint/velocity");
}

TEST_F(
  TestHardwareSystemInterface4WS4WD,
  checkCommandValuesArePackedIntoJointState)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto command_interfaces = system_interface.export_command_interfaces();
  for (size_t n = 0; n < 8; ++n) {
  (void)command_interfaces[n].set_value(n + 1.0);
  }

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WS4WD>("mobile_base");
  auto command = interface.get_joint_state_command();

  ASSERT_EQ(command.name.size(), 8u);
  EXPECT_DOUBLE_EQ(command.position[0], 1.0);
  EXPECT_DOUBLE_EQ(command.position[1], 2.0);
  EXPECT_DOUBLE_EQ(command.position[2], 3.0);
  EXPECT_DOUBLE_EQ(command.position[3], 4.0);
  EXPECT_DOUBLE_EQ(command.velocity[4], 5.0);
  EXPECT_DOUBLE_EQ(command.velocity[5], 6.0);
  EXPECT_DOUBLE_EQ(command.velocity[6], 7.0);
  EXPECT_DOUBLE_EQ(command.velocity[7], 8.0);
}

TEST_F(
  TestHardwareSystemInterface4WS4WD,
  checkJointStateFeedbackValuesAreExportedIntoStateInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WS4WD>("mobile_base");

  auto feedback = romea::ros2::make_joint_state_msg(8);
  feedback.name[0] = "front_left_wheel_steering_joint";
  feedback.name[1] = "front_right_wheel_steering_joint";
  feedback.name[2] = "rear_left_wheel_steering_joint";
  feedback.name[3] = "rear_right_wheel_steering_joint";
  feedback.name[4] = "front_left_wheel_spinning_joint";
  feedback.name[5] = "front_right_wheel_spinning_joint";
  feedback.name[6] = "rear_left_wheel_spinning_joint";
  feedback.name[7] = "rear_right_wheel_spinning_joint";
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

  interface.set_feedback(feedback);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 16u);
  for (size_t n = 0; n < 16; ++n) {
    EXPECT_DOUBLE_EQ(state_interfaces[n].get_value(), n + 1.0);
  }
}

TEST_F(
  TestHardwareSystemInterface4WS4WD,
  checkCommandValuesAreAvailableAsCoreHardwareCommand)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto command_interfaces = system_interface.export_command_interfaces();
  for (size_t n = 0; n < 8; ++n) {
  (void)command_interfaces[n].set_value(n + 1.0);
  }

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WS4WD>("mobile_base");
  auto command = interface.get_hardware_command();

  EXPECT_DOUBLE_EQ(command.frontLeftWheelSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(command.frontRightWheelSteeringAngle, 2.0);
  EXPECT_DOUBLE_EQ(command.rearLeftWheelSteeringAngle, 3.0);
  EXPECT_DOUBLE_EQ(command.rearRightWheelSteeringAngle, 4.0);
  EXPECT_DOUBLE_EQ(command.frontLeftWheelSpinningSetPoint, 5.0);
  EXPECT_DOUBLE_EQ(command.frontRightWheelSpinningSetPoint, 6.0);
  EXPECT_DOUBLE_EQ(command.rearLeftWheelSpinningSetPoint, 7.0);
  EXPECT_DOUBLE_EQ(command.rearRightWheelSpinningSetPoint, 8.0);
}

TEST_F(
  TestHardwareSystemInterface4WS4WD,
  checkCoreHardwareStateValuesAreExportedIntoStateInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface4WS4WD>("mobile_base");

  romea::core::HardwareState4WS4WD state;
  state.frontLeftWheelSteeringAngle = 1.0;
  state.frontRightWheelSteeringAngle = 2.0;
  state.rearLeftWheelSteeringAngle = 3.0;
  state.rearRightWheelSteeringAngle = 4.0;
  state.frontLeftWheelSpinningMotion.position = 5.0;
  state.frontLeftWheelSpinningMotion.velocity = 6.0;
  state.frontLeftWheelSpinningMotion.torque = 7.0;
  state.frontRightWheelSpinningMotion.position = 8.0;
  state.frontRightWheelSpinningMotion.velocity = 9.0;
  state.frontRightWheelSpinningMotion.torque = 10.0;
  state.rearLeftWheelSpinningMotion.position = 11.0;
  state.rearLeftWheelSpinningMotion.velocity = 12.0;
  state.rearLeftWheelSpinningMotion.torque = 13.0;
  state.rearRightWheelSpinningMotion.position = 14.0;
  state.rearRightWheelSpinningMotion.velocity = 15.0;
  state.rearRightWheelSpinningMotion.torque = 16.0;

  interface.set_feedback(state);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 16u);
  for (size_t n = 0; n < 16; ++n) {
    EXPECT_DOUBLE_EQ(state_interfaces[n].get_value(), n + 1.0);
  }
}
