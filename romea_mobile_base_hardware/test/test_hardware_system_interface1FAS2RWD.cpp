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
#include "romea_mobile_base_hardware/hardware_interface1FAS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface_base.hpp"
#include "test_utils.hpp"

class TestHardwareSystemInterface1FAS2RWD : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_infos = parse_hardware_info("test_hardware_system_interface1FAS2RWD.xacro");
    ASSERT_EQ(hardware_infos.size(), 1u);
  }

  std::vector<hardware_interface::HardwareInfo> hardware_infos;
};

TEST_F(
  TestHardwareSystemInterface1FAS2RWD,
  checkFactoryCreatesInterfaceFromPrefixedParameters)
{
  auto hardware_interface =
    romea::ros2::make_hardware_interface(hardware_infos[0], "1FAS2RWD", "mobile_base");

  auto & interface = dynamic_cast<romea::ros2::HardwareInterface1FAS2RWD &>(*hardware_interface);

  auto joint_names = interface.get_joint_names();
  ASSERT_EQ(joint_names.size(), 3u);
  EXPECT_EQ(joint_names[0], "front_axle_steering_joint");
  EXPECT_EQ(joint_names[1], "rear_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[2], "rear_right_wheel_spinning_joint");
}

TEST_F(
  TestHardwareSystemInterface1FAS2RWD,
  checkSystemInterfaceLoadsDynamicInterface)
{
  TestableHardwareSystemInterface system_interface;

  EXPECT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface1FAS2RWD>("mobile_base");

  auto joint_names = interface.get_joint_names();
  ASSERT_EQ(joint_names.size(), 3u);
  EXPECT_EQ(joint_names[0], "front_axle_steering_joint");
  EXPECT_EQ(joint_names[1], "rear_left_wheel_spinning_joint");
  EXPECT_EQ(joint_names[2], "rear_right_wheel_spinning_joint");
}

TEST_F(
  TestHardwareSystemInterface1FAS2RWD,
  checkSystemInterfaceExportsStateAndCommandInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 15u);
  expect_interface_name(state_interfaces[0], "front_axle_steering_joint/position");
  expect_interface_name(state_interfaces[1], "rear_left_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[4], "rear_right_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[7], "front_left_wheel_steering_joint/position");
  expect_interface_name(state_interfaces[8], "front_right_wheel_steering_joint/position");
  expect_interface_name(state_interfaces[9], "front_left_wheel_spinning_joint/position");
  expect_interface_name(state_interfaces[12], "front_right_wheel_spinning_joint/position");

  auto command_interfaces = system_interface.export_command_interfaces();
  ASSERT_EQ(command_interfaces.size(), 3u);
  expect_interface_name(command_interfaces[0], "front_axle_steering_joint/position");
  expect_interface_name(command_interfaces[1], "rear_left_wheel_spinning_joint/velocity");
  expect_interface_name(command_interfaces[2], "rear_right_wheel_spinning_joint/velocity");
}

TEST_F(
  TestHardwareSystemInterface1FAS2RWD,
  checkCommandValuesArePackedIntoJointState)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto command_interfaces = system_interface.export_command_interfaces();
  (void)command_interfaces[0].set_value(1.0);
  (void)command_interfaces[1].set_value(2.0);
  (void)command_interfaces[2].set_value(3.0);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface1FAS2RWD>("mobile_base");
  auto command = interface.get_joint_state_command();

  ASSERT_EQ(command.name.size(), 3u);
  EXPECT_EQ(command.name[0], "front_axle_steering_joint");
  EXPECT_EQ(command.name[1], "rear_left_wheel_spinning_joint");
  EXPECT_EQ(command.name[2], "rear_right_wheel_spinning_joint");

  EXPECT_DOUBLE_EQ(command.position[0], 1.0);
  EXPECT_DOUBLE_EQ(command.velocity[1], 2.0);
  EXPECT_DOUBLE_EQ(command.velocity[2], 3.0);
}

TEST_F(
  TestHardwareSystemInterface1FAS2RWD,
  checkJointStateFeedbackValuesAreExportedIntoStateInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface1FAS2RWD>("mobile_base");

  auto feedback = romea::ros2::make_joint_state_msg(3);
  feedback.name[0] = "front_axle_steering_joint";
  feedback.name[1] = "rear_left_wheel_spinning_joint";
  feedback.name[2] = "rear_right_wheel_spinning_joint";
  feedback.position[0] = 1.0;
  feedback.position[1] = 2.0;
  feedback.velocity[1] = 3.0;
  feedback.effort[1] = 4.0;
  feedback.position[2] = 5.0;
  feedback.velocity[2] = 6.0;
  feedback.effort[2] = 7.0;

  interface.set_feedback(feedback);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 15u);
  for (size_t n = 0; n < 7; ++n) {
    EXPECT_DOUBLE_EQ(state_interfaces[n].get_value(), n + 1.0);
  }
}

TEST_F(
  TestHardwareSystemInterface1FAS2RWD,
  checkCommandValuesAreAvailableAsCoreHardwareCommand)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto command_interfaces = system_interface.export_command_interfaces();
  (void)command_interfaces[0].set_value(1.0);
  (void)command_interfaces[1].set_value(2.0);
  (void)command_interfaces[2].set_value(3.0);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface1FAS2RWD>("mobile_base");
  auto command = interface.get_hardware_command();

  EXPECT_DOUBLE_EQ(command.frontAxleSteeringAngle, 1.0);
  EXPECT_DOUBLE_EQ(command.rearLeftWheelSpinningSetPoint, 2.0);
  EXPECT_DOUBLE_EQ(command.rearRightWheelSpinningSetPoint, 3.0);
}

TEST_F(
  TestHardwareSystemInterface1FAS2RWD,
  checkCoreHardwareStateValuesAreExportedIntoStateInterfaces)
{
  TestableHardwareSystemInterface system_interface;
  ASSERT_EQ(
    system_interface.on_init(hardware_infos[0]),
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

  auto & interface =
    system_interface.hardware_interface<romea::ros2::HardwareInterface1FAS2RWD>("mobile_base");

  romea::core::HardwareState1FAS2RWD state;
  state.frontAxleSteeringAngle = 1.0;
  state.rearLeftWheelSpinningMotion.position = 2.0;
  state.rearLeftWheelSpinningMotion.velocity = 3.0;
  state.rearLeftWheelSpinningMotion.torque = 4.0;
  state.rearRightWheelSpinningMotion.position = 5.0;
  state.rearRightWheelSpinningMotion.velocity = 6.0;
  state.rearRightWheelSpinningMotion.torque = 7.0;

  interface.set_feedback(state);

  auto state_interfaces = system_interface.export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 15u);
  for (size_t n = 0; n < 7; ++n) {
    EXPECT_DOUBLE_EQ(state_interfaces[n].get_value(), n + 1.0);
  }
}
