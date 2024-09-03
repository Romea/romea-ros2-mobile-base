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
#include <string>
#include <memory>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/node.hpp"
#include "hardware_interface/component_parser.hpp"

// romea
#include "../test/test_helper.h"
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2ASxxx.hpp"

class TestHardwareInfo2ASxxx : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_info2ASxxx.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface2ASxxx.urdf";
    std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
    std::system(cmd.c_str());

    std::ifstream file(urdf_file.c_str());
    std::stringstream buffer;
    buffer << file.rdbuf();
    // std::cout << buffer.str() << std::endl;

    info = hardware_interface::parse_control_resources_from_urdf(buffer.str());
  }

  std::vector<hardware_interface::HardwareInfo> info;
};


TEST_F(TestHardwareInfo2ASxxx, checkJointsInfo)
{
  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_front_axle_steering_joint_info(
      info[0]).name.c_str(), "robot_joint1");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_rear_axle_steering_joint_info(
      info[0]).name.c_str(), "robot_joint2");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_front_left_wheel_steering_joint_info(
      info[0]).name.c_str(), "robot_joint3");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_front_right_wheel_steering_joint_info(
      info[0]).name.c_str(), "robot_joint4");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_rear_left_wheel_steering_joint_info(
      info[0]).name.c_str(), "robot_joint5");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_rear_right_wheel_steering_joint_info(
      info[0]).name.c_str(), "robot_joint6");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_front_left_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint7");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_front_right_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint8");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_rear_left_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint9");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2ASxxx::get_rear_right_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint10");
}

TEST_F(TestHardwareInfo2ASxxx, checkHardwareParameters)
{
  EXPECT_DOUBLE_EQ(romea::ros2::get_wheelbase(info[0]), 1.3);
  EXPECT_DOUBLE_EQ(romea::ros2::get_front_track(info[0]), 1.2);
  EXPECT_DOUBLE_EQ(romea::ros2::get_front_wheel_radius(info[0]), 0.4);
  EXPECT_DOUBLE_EQ(romea::ros2::get_front_hub_carrier_offset(info[0]), 0.1);
  EXPECT_DOUBLE_EQ(romea::ros2::get_rear_track(info[0]), 1.2);
  EXPECT_DOUBLE_EQ(romea::ros2::get_rear_wheel_radius(info[0]), 0.6);
  EXPECT_DOUBLE_EQ(romea::ros2::get_rear_hub_carrier_offset(info[0]), 0.1);
}
//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
