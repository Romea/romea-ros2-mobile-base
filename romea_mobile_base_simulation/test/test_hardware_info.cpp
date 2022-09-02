//gtest
#include <gtest/gtest.h>
#include "test_helper.h"

//ros
#include <rclcpp/node.hpp>

//romea
#include "romea_mobile_base_hardware/hardware_info.hpp"
#include <hardware_interface/component_parser.hpp>

class TestHarwareInfo : public ::testing::Test
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
         <param name="foo">3.14</param>
         <param name="baz">string</param>
       </hardware>
      <joint name="joint1">
        <command_interface name="position">
          <min>-1.</min>
          <min> 1.</min>
        </command_interface>
        <state_interface name="position"/>
      </joint>
      <joint name="joint2">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <state_interface name="position">
           <min>-1.</min>
           <min> 1.</min>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
     </ros2_control>
   </robot>
   )";

   info = hardware_interface::parse_control_resources_from_urdf(urdf)[0];
  }
  hardware_interface::HardwareInfo info;
};


TEST_F(TestHarwareInfo, getJointInfo)
{
  auto joint_info =romea::get_joint_info(info,"joint1");
  EXPECT_STREQ(joint_info.name.c_str(),"joint1");
}

TEST_F(TestHarwareInfo, failedToGetJointInfo)
{
  EXPECT_THROW(romea::get_joint_info(info,"joint3"),std::runtime_error);
}

TEST_F(TestHarwareInfo, getStateInterfaceInfo)
{
  auto joint_info =romea::get_joint_info(info,"joint2");
  auto interface_info = romea::get_state_interface_info(joint_info,"position");
  EXPECT_STREQ(interface_info.name.c_str(),"position");
}

TEST_F(TestHarwareInfo, failedToGetStateInterfaceInfo)
{
  auto joint_info =romea::get_joint_info(info,"joint1");
  EXPECT_THROW(romea::get_state_interface_info(joint_info,"velocity"),std::runtime_error);
}

TEST_F(TestHarwareInfo, getCommandInterfaceInfo)
{
  auto joint_info =romea::get_joint_info(info,"joint2");
  auto interface_info = romea::get_command_interface_info(joint_info,"velocity");
  EXPECT_STREQ(interface_info.name.c_str(),"velocity");
}

TEST_F(TestHarwareInfo, getInterfaceMinMax)
{
  auto joint_info =romea::get_joint_info(info,"joint1");
  auto interface_info = romea::get_state_interface_info(joint_info,"position");
  EXPECT_DOUBLE_EQ(romea::get_min((interface_info)),-std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(romea::get_max((interface_info)), std::numeric_limits<double>::max());
}

TEST_F(TestHarwareInfo, getInterfaceMinMaxWhenTheyAreNotDefined)
{
  auto joint_info =romea::get_joint_info(info,"joint1");
  auto interface_info = romea::get_command_interface_info(joint_info,"position");
  EXPECT_DOUBLE_EQ(romea::get_min((interface_info)),-std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(romea::get_max((interface_info)), std::numeric_limits<double>::max());
}


TEST_F(TestHarwareInfo, failedToGetCommandInterfaceInfo)
{
  auto joint_info =romea::get_joint_info(info,"joint1");
  EXPECT_THROW(romea::get_command_interface_info(joint_info,"velocity"),std::runtime_error);
}

TEST_F(TestHarwareInfo, checkHasParemeter)
{
  EXPECT_TRUE(romea::has_parameter(info,"foo"));
  EXPECT_FALSE(romea::has_parameter(info,"bar"));
}

TEST_F(TestHarwareInfo, checkGetParemeter)
{
   EXPECT_DOUBLE_EQ(romea::get_parameter<double>(info,"foo"),3.14);
   EXPECT_STREQ(romea::get_parameter<std::string>(info,"baz").c_str(),"string");
}

TEST_F(TestHarwareInfo, checkFailedToGetParemeter)
{
  EXPECT_THROW(romea::get_parameter(info,"bar"),std::out_of_range);
  EXPECT_THROW(romea::get_parameter<double>(info,"baz"),std::invalid_argument);
}

TEST_F(TestHarwareInfo, checkGetParemeterOr)
{
   EXPECT_DOUBLE_EQ(romea::get_parameter_or<double>(info,"foo",2.71),3.14);
   EXPECT_DOUBLE_EQ(romea::get_parameter_or<double>(info,"bar",2.71),2.71);
}