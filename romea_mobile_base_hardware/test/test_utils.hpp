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

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

// std
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// gtest
#include "gtest/gtest.h"

// ros
#include "hardware_interface/component_parser.hpp"
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include "test_helper.h"  // NOLINT

inline std::vector<hardware_interface::HardwareInfo> parse_hardware_info(
  const std::string & xacro_name)
{
  const std::string xacro_file = std::string(TEST_DIR) + "/" + xacro_name;
  const std::string urdf_file = "/tmp/" + xacro_name + ".urdf";
  const std::string cmd = "xacro " + xacro_file + " > " + urdf_file;
  std::system(cmd.c_str());

  std::ifstream file(urdf_file.c_str());
  std::stringstream buffer;
  buffer << file.rdbuf();

  return hardware_interface::parse_control_resources_from_urdf(buffer.str());
}

template<typename Interface>
void expect_interface_name(const Interface & interface, const std::string & expected_name)
{
  EXPECT_STREQ(interface.get_name().c_str(), expected_name.c_str());
}

class TestableHardwareSystemInterface : public romea::ros2::HardwareSystemInterface
{
public:
  using HardwareSystemInterface::hardware_interface;

  TestableHardwareSystemInterface()
  : HardwareSystemInterface("TestableHardwareSystemInterface")
  {
  }

private:
  hardware_interface::return_type connect_() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type disconnect_() override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }
};

#endif  // TEST_UTILS_HPP_
