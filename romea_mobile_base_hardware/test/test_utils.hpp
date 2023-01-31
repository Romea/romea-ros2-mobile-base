// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

// std
#include <string>

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

#endif  // TEST_UTILS_HPP_
