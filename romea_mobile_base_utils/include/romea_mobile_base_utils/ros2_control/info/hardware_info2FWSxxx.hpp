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


#ifndef ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__HARDWARE_INFO2FWSXXX_HPP_
#define ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__HARDWARE_INFO2FWSXXX_HPP_


// ros
#include "romea_mobile_base_utils/ros2_control/info/hardware_info_common.hpp"


namespace romea
{
namespace ros2
{

struct HardwareInfo2FWSxxx
{
  static hardware_interface::ComponentInfo get_front_left_wheel_steering_joint_info(
    const hardware_interface::HardwareInfo & hardware_info);

  static hardware_interface::ComponentInfo get_front_right_wheel_steering_joint_info(
    const hardware_interface::HardwareInfo & hardware_info);

  static hardware_interface::ComponentInfo get_front_left_wheel_spinning_joint_info(
    const hardware_interface::HardwareInfo & hardware_info);

  static hardware_interface::ComponentInfo get_front_right_wheel_spinning_joint_info(
    const hardware_interface::HardwareInfo & hardware_info);

  static hardware_interface::ComponentInfo get_rear_left_wheel_spinning_joint_info(
    const hardware_interface::HardwareInfo & hardware_info);

  static hardware_interface::ComponentInfo get_rear_right_wheel_spinning_joint_info(
    const hardware_interface::HardwareInfo & hardware_info);
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__ROS2_CONTROL__INFO__HARDWARE_INFO2FWSXXX_HPP_
