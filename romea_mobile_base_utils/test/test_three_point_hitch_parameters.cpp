// Copyright 2026 INRAE, French National Research Institute for Agriculture,
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

// std
#include <limits>
#include <memory>

// gtest
#include "gtest/gtest.h"

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_utils/params/three_point_hitch_parameters.hpp"

TEST(TestThreePointHitchParameters, GetsInfo)
{
  const std::string ns = "hitch";
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {ns + ".geometry.lower_link_length", 1.0},
      {ns + ".geometry.lower_links_base_spacing", 2.0},
      {ns + ".geometry.cylinders_x_offset", 3.0},
      {ns + ".geometry.cylinders_z_offset", 4.0},
      {ns + ".geometry.cylinders_base_spacing", 5.0},
      {ns + ".geometry.cylinders_rod_attachment_distance", 6.0},
      {ns + ".geometry.cylinders_rod_attachment_height", 7.0},
      {ns + ".geometry.cylinders_dead_length", 8.0},
      {ns + ".geometry.cylinders_stroke", 9.0},
      {ns + ".geometry.upper_link_x_offset", 10.0},
      {ns + ".geometry.upper_link_z_offset", 11.0},
      {ns + ".geometry.upper_link_length", 12.0},
      {ns + ".control.command.minimal_length", 0.2},
      {ns + ".control.command.maximal_length", 0.4},
      {ns + ".control.command.maximal_speed", 0.03},
    });
  const auto node = std::make_shared<rclcpp::Node>(
    "test_three_point_hitch_parameters", options);

  romea::ros2::declare_three_point_hitch_info(node, ns);

  const auto info = romea::ros2::get_three_point_hitch_info(node, ns);

  EXPECT_DOUBLE_EQ(info.geometry.lower_link_length, 1.0);
  EXPECT_DOUBLE_EQ(info.geometry.cylinders_dead_length, 8.0);
  EXPECT_DOUBLE_EQ(info.geometry.cylinders_stroke, 9.0);
  EXPECT_DOUBLE_EQ(info.geometry.upper_link_length, 12.0);
  EXPECT_DOUBLE_EQ(info.control.sensor.position_std, 0.0);
  EXPECT_DOUBLE_EQ(info.control.command.minimal_length, 0.2);
  EXPECT_DOUBLE_EQ(info.control.command.maximal_length, 0.4);
  EXPECT_DOUBLE_EQ(info.control.command.maximal_speed, 0.03);
  EXPECT_DOUBLE_EQ(
    info.control.command.maximal_linear_acceleration,
    std::numeric_limits<double>::max());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
