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
#include <string>

// local
#include "romea_mobile_base_controllers/interfaces/controller_interface_common.hpp"

namespace romea
{
namespace ros2
{

std::string hardware_position_interface_name(const std::string joint_name)
{
  return joint_name + "/" + hardware_interface::HW_IF_POSITION;
}

std::string hardware_velocity_interface_name(const std::string joint_name)
{
  return joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
}

std::string hardware_effort_interface_name(const std::string joint_name)
{
  return joint_name + "/" + hardware_interface::HW_IF_EFFORT;
}

}  // namespace ros2
}  // namespace romea
