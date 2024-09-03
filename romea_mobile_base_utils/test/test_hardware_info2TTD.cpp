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
#include "romea_mobile_base_utils/ros2_control/info/hardware_info2TTD.hpp"

class TestHardwareInfo2TTD : public ::testing::Test
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
    std::string xacro_file = std::string(TEST_DIR) + "/test_hardware_info2TTD.xacro";
    std::string urdf_file = "/tmp/test_simulation_interface2TTD.urdf";
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


TEST_F(TestHardwareInfo2TTD, checkJointsInfo)
{
  EXPECT_STREQ(
    romea::ros2::HardwareInfo2TTD::get_left_sprocket_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint1");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2TTD::get_right_sprocket_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint2");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2TTD::get_left_idler_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint3");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2TTD::get_right_idler_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint4");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2TTD::get_front_left_roller_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint5");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2TTD::get_front_right_roller_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint6");
  EXPECT_STREQ(
    romea::ros2::HardwareInfo2TTD::get_rear_left_roller_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint7");

  EXPECT_STREQ(
    romea::ros2::HardwareInfo2TTD::get_rear_right_roller_wheel_spinning_joint_info(
      info[0]).name.c_str(), "robot_joint8");
}

TEST_F(TestHardwareInfo2TTD, checkHardwareParameters)
{
  EXPECT_DOUBLE_EQ(romea::ros2::get_sprocket_wheel_radius(info[0]), 0.8);
  EXPECT_DOUBLE_EQ(romea::ros2::get_idler_wheel_radius(info[0]), 0.3);
  EXPECT_DOUBLE_EQ(romea::ros2::get_roller_wheel_radius(info[0]), 0.2);
  EXPECT_DOUBLE_EQ(romea::ros2::get_track_thickness(info[0]), 0.1);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
