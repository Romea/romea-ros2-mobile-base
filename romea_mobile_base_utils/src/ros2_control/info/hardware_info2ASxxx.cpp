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
// limitations under the Licens

#include "romea_mobile_base_utils/ros2_control/info/hardware_info2ASxxx.hpp"

namespace
{
const size_t FRONT_AXLE_STEERING_JOINT_ID = 0;
const size_t REAR_AXLE_STEERING_JOINT_ID = 1;
const size_t FRONT_LEFT_WHEEL_STEERING_JOINT_ID = 2;
const size_t FRONT_RIGHT_WHEEL_STEERING_JOINT_ID = 3;
const size_t REAR_LEFT_WHEEL_STEERING_JOINT_ID = 4;
const size_t REAR_RIGHT_WHEEL_STEERING_JOINT_ID = 5;
const size_t FRONT_LEFT_WHEEL_SPINNING_JOINT_ID = 6;
const size_t FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID = 7;
const size_t REAR_LEFT_WHEEL_SPINNING_JOINT_ID = 8;
const size_t REAR_RIGHT_WHEEL_SPINNING_JOINT_ID = 9;
}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_front_axle_steering_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[FRONT_AXLE_STEERING_JOINT_ID];
}

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_rear_axle_steering_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[REAR_AXLE_STEERING_JOINT_ID];
}

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_front_left_wheel_steering_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[FRONT_LEFT_WHEEL_STEERING_JOINT_ID];
}


//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_front_right_wheel_steering_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[FRONT_RIGHT_WHEEL_STEERING_JOINT_ID];
}

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_rear_left_wheel_steering_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[REAR_LEFT_WHEEL_STEERING_JOINT_ID];
}


//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_rear_right_wheel_steering_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[REAR_RIGHT_WHEEL_STEERING_JOINT_ID];
}

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_front_left_wheel_spinning_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[FRONT_LEFT_WHEEL_SPINNING_JOINT_ID];
}

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_front_right_wheel_spinning_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID];
}

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_rear_left_wheel_spinning_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[REAR_LEFT_WHEEL_SPINNING_JOINT_ID];
}

//-----------------------------------------------------------------------------
hardware_interface::ComponentInfo HardwareInfo2ASxxx::get_rear_right_wheel_spinning_joint_info(
  const hardware_interface::HardwareInfo & hardware_info)
{
  return hardware_info.joints[REAR_RIGHT_WHEEL_SPINNING_JOINT_ID];
}


}  // namespace ros2
}  // namespace romea
