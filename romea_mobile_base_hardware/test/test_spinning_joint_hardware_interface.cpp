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
#include <string>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"
#include "hardware_interface/component_parser.hpp"

// romea
#include "../test/test_helper.h"
#include "../test/test_utils.hpp"
#include "romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp"

class TestSpinningJointHardwateInterface : public ::testing::Test
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
    joint_info.name = "spinning_wheel";
    joint_info.type = "joint";
    joint_info.command_interfaces.push_back(
      make_interface_info(
        hardware_interface::HW_IF_VELOCITY,
        "-1", "1"));
    joint_info.command_interfaces.push_back(
      make_interface_info(
        hardware_interface::HW_IF_EFFORT,
        "-1", "1"));
    joint_info.state_interfaces.push_back(
      make_interface_info(
        hardware_interface::HW_IF_POSITION,
        "", ""));
    joint_info.state_interfaces.push_back(
      make_interface_info(
        hardware_interface::HW_IF_VELOCITY,
        "", ""));
    joint_info.state_interfaces.push_back(
      make_interface_info(
        hardware_interface::HW_IF_EFFORT, "",
        ""));
  }

  void MakeJoint(const std::string & command_interface_type)
  {
    joint = std::make_unique<romea::ros2::SpinningJointHardwareInterface>(
      joint_info,
      command_interface_type);
  }

  std::unique_ptr<romea::ros2::SpinningJointHardwareInterface> joint;
  hardware_interface::ComponentInfo joint_info;
};


TEST_F(TestSpinningJointHardwateInterface, checkExportedStateInterfaces)
{
  MakeJoint(hardware_interface::HW_IF_VELOCITY);
  std::vector<hardware_interface::StateInterface> state_interfaces;
  joint->export_state_interfaces(state_interfaces);

  EXPECT_EQ(state_interfaces.size(), 3u);
  check_interface_name(state_interfaces[0], "spinning_wheel/position");
  check_interface_name(state_interfaces[1], "spinning_wheel/velocity");
  check_interface_name(state_interfaces[2], "spinning_wheel/effort");
}

TEST_F(TestSpinningJointHardwateInterface, checkExportedCommandInterfaceWhenVelocityControlIsUsed)
{
  MakeJoint(hardware_interface::HW_IF_VELOCITY);
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  joint->export_command_interface(command_interfaces);

  EXPECT_EQ(command_interfaces.size(), 1u);
  check_interface_name(command_interfaces[0], "spinning_wheel/velocity");
}

TEST_F(TestSpinningJointHardwateInterface, checkExportedCommandInterfaceWhenEffortControlIsUsed)
{
  MakeJoint(hardware_interface::HW_IF_EFFORT);
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  joint->export_command_interface(command_interfaces);

  EXPECT_EQ(command_interfaces.size(), 1u);
  check_interface_name(command_interfaces[0], "spinning_wheel/effort");
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
