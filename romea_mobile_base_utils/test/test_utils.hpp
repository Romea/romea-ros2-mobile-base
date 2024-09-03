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


#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

// std
#include <string>

// ros
#include "hardware_interface/hardware_info.hpp"

// romea
#include "romea_common_utils/ros_versions.hpp"

template<typename Interface>
void check_interface_name(
  const Interface & interface,
  const std::string & expected_name)
{
#if ROS_DISTRO == ROS_GALACTIC
  EXPECT_STREQ(interface.get_full_name().c_str(), expected_name.c_str());
#else
  EXPECT_STREQ(interface.get_name().c_str(), expected_name.c_str());
#endif
}

hardware_interface::InterfaceInfo make_interface_info(
  const std::string & name,
  const std::string & min,
  const std::string & max)
{
  hardware_interface::InterfaceInfo info;
  info.name = name;
  info.min = min;
  info.max = max;
  return info;
}


#endif  // TEST_UTILS_HPP_
