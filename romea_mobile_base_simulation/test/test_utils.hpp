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
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ros
#include "hardware_interface/component_parser.hpp"

// romea
#include "romea_mobile_base_simulation/generic_simulation_system_interface.hpp"
#include "test_helper.h"  // NOLINT

std::vector<hardware_interface::HardwareInfo> parse_hardware_info(const std::string & xacro_name)
{
  const std::string xacro_file = std::string(TEST_DIR) + "/" + xacro_name;
  const std::string urdf_file = "/tmp/" + xacro_name + ".urdf";
  const std::string command = "xacro " + xacro_file + " > " + urdf_file;
  if (std::system(command.c_str()) != 0) {
    throw std::runtime_error("Cannot generate URDF from " + xacro_file);
  }

  std::ifstream file(urdf_file.c_str());
  std::stringstream buffer;
  buffer << file.rdbuf();
  return hardware_interface::parse_control_resources_from_urdf(buffer.str());
}

void expect_interface_name(
  const hardware_interface::Handle & interface, const std::string & expected_name)
{
  EXPECT_EQ(interface.get_prefix_name() + "/" + interface.get_interface_name(), expected_name);
}

class TestableGenericSimulationSystemInterface
: public romea::ros2::GenericSimulationSystemInterface
{
public:
  romea::ros2::SimulationInterfaceBase & simulation_interface(const std::string & name)
  {
    return *simulation_interfaces_.at(name);
  }
};

#endif  // TEST_UTILS_HPP_
