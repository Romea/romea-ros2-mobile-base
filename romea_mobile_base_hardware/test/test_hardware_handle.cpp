// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"

// romea
#include "hardware_interface/component_parser.hpp"

// local
#include "../test/test_helper.h"
#include "romea_mobile_base_hardware/hardware_handle.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

class TestHardwareInterfaceHandle : public ::testing::Test
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
    std::string urdf =
      R"(
 <?xml version="1.0" encoding="utf-8"?>
 <!-- =================================================================================== -->
 <!-- |    This document was autogenerated by xacro from minimal_robot.urdf.xacro       | -->
 <!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
 <!-- =================================================================================== -->
 <robot name="robot">
   <ros2_control name="base" type="system">
     <hardware>
       <plugin>fake_components/GenericSystem</plugin>
     </hardware>
    <joint name="joint1">
      <command_interface name="position">
        <min>-1.</min>
        <min> 1.</min>
      </command_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="effort">
         <min>-1.</min>
         <min> 1.</min>
      </state_interface>
    </joint>
   </ros2_control>
 </robot>
 )";

    info = hardware_interface::parse_control_resources_from_urdf(urdf)[0];
  }
  hardware_interface::HardwareInfo info;
};


TEST_F(TestHardwareInterfaceHandle, checkMakeHardwareCommandInterface)
{
  auto joint_info = romea::get_joint_info(info, "joint1");
  auto interface = romea::HardwareCommandInterface(joint_info, "position");

  EXPECT_STREQ(interface.get_joint_name().c_str(), "joint1");
  EXPECT_STREQ(interface.get_interface_type().c_str(), "position");
}

TEST_F(TestHardwareInterfaceHandle, failedToMakeHardwareCommandInterface)
{
  auto joint_info = romea::get_joint_info(info, "joint1");
  EXPECT_THROW(romea::HardwareCommandInterface(joint_info, "effort"), std::runtime_error);
}

TEST_F(TestHardwareInterfaceHandle, checkMakeHardwareStateInterface)
{
  auto joint_info = romea::get_joint_info(info, "joint2");
  auto interface = romea::HardwareStateInterface(joint_info, "effort");

  EXPECT_STREQ(interface.get_joint_name().c_str(), "joint2");
  EXPECT_STREQ(interface.get_interface_type().c_str(), "effort");
}

TEST_F(TestHardwareInterfaceHandle, failedToMakeHardwareStateInterface)
{
  auto joint_info = romea::get_joint_info(info, "joint2");
  EXPECT_THROW(romea::HardwareStateInterface(joint_info, "position"), std::runtime_error);
}
