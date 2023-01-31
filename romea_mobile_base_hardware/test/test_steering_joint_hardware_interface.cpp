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
#include "../test/test_utils.hpp"
#include "romea_mobile_base_hardware/steering_joint_hardware_interface.hpp"

class TestSteeringJointHardwateInterface : public ::testing::Test
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
    joint_info.name = "steering_wheel";
    joint_info.type = "joint";
    joint_info.command_interfaces.push_back({hardware_interface::HW_IF_POSITION, "-1", "1", "", 0});
    joint_info.state_interfaces.push_back({hardware_interface::HW_IF_POSITION, "", "", "", 0});
    joint = std::make_unique<romea::SteeringJointHardwareInterface>(joint_info);
  }


  std::unique_ptr<romea::SteeringJointHardwareInterface> joint;
  hardware_interface::ComponentInfo joint_info;
};

TEST_F(TestSteeringJointHardwateInterface, checkExportedStateInterfaces)
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  joint->export_state_interface(state_interfaces);

  EXPECT_EQ(state_interfaces.size(), 1u);
  check_interface_name(state_interfaces[0], "steering_wheel/position");
}

TEST_F(TestSteeringJointHardwateInterface, checkExportedCommandInterfaces)
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  joint->export_command_interface(command_interfaces);

  EXPECT_EQ(command_interfaces.size(), 1u);
  check_interface_name(command_interfaces[0], "steering_wheel/position");
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
