// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

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
    joint_info.command_interfaces.push_back({hardware_interface::HW_IF_VELOCITY, "-1", "1", "", 0});
    joint_info.command_interfaces.push_back({hardware_interface::HW_IF_EFFORT, "-1", "1", "", 0});
    joint_info.state_interfaces.push_back({hardware_interface::HW_IF_POSITION, "", "", "", 0});
    joint_info.state_interfaces.push_back({hardware_interface::HW_IF_VELOCITY, "", "", "", 0});
    joint_info.state_interfaces.push_back({hardware_interface::HW_IF_EFFORT, "", "", "", 0});
  }

  void MakeJoint(const std::string & command_interface_type)
  {
    joint = std::make_unique<romea::SpinningJointHardwareInterface>(
      joint_info,
      command_interface_type);
  }

  std::unique_ptr<romea::SpinningJointHardwareInterface> joint;
  hardware_interface::ComponentInfo joint_info;
};


TEST_F(TestSpinningJointHardwateInterface, checkExportedStateInterfaces)
{
  MakeJoint(hardware_interface::HW_IF_VELOCITY);
  std::vector<hardware_interface::StateInterface> state_interfaces;
  joint->export_state_interfaces(state_interfaces);

  EXPECT_EQ(state_interfaces.size(), 3u);
  EXPECT_STREQ(state_interfaces[0].get_full_name().c_str(), "spinning_wheel/position");
  EXPECT_STREQ(state_interfaces[1].get_full_name().c_str(), "spinning_wheel/velocity");
  EXPECT_STREQ(state_interfaces[2].get_full_name().c_str(), "spinning_wheel/effort");
}

TEST_F(TestSpinningJointHardwateInterface, checkExportedCommandInterfaceWhenVelocityControlIsUsed)
{
  MakeJoint(hardware_interface::HW_IF_VELOCITY);
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  joint->export_command_interface(command_interfaces);

  EXPECT_EQ(command_interfaces.size(), 1u);
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "spinning_wheel/velocity");
}

TEST_F(TestSpinningJointHardwateInterface, checkExportedCommandInterfaceWhenEffortControlIsUsed)
{
  MakeJoint(hardware_interface::HW_IF_EFFORT);
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  joint->export_command_interface(command_interfaces);

  EXPECT_EQ(command_interfaces.size(), 1u);
  EXPECT_STREQ(command_interfaces[0].get_full_name().c_str(), "spinning_wheel/effort");
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
