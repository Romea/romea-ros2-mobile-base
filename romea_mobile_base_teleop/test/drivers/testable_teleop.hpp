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

#ifndef DRIVERS__TESTABLE_TELEOP_HPP_
#define DRIVERS__TESTABLE_TELEOP_HPP_

// std
#include <map>
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

template<class TeleopType>
class TestableTeleop : public TeleopType
{
public:
  explicit TestableTeleop(const rclcpp::NodeOptions & options) : TeleopType(options) {}

  std::shared_ptr<rclcpp::Node> get_node() const { return TeleopType::node_; }

  std::map<std::string, int> get_mapping() const { return TeleopType::joy_->get_mapping(); }
};

#endif  // DRIVERS__TESTABLE_TELEOP_HPP_
