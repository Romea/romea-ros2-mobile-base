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


#ifndef ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_TRAITS_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_TRAITS_HPP_

#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringCommand.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/OneAxleSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/TwoAxleSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/OmniSteeringMeasure.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringMeasure.hpp>
#include <romea_mobile_base_msgs/msg/omni_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/one_axle_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/two_axle_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/skid_steering_measure_stamped.hpp>

namespace romea
{
namespace ros2
{

template<typename CommandType>
struct CommandTraits;

template<>
struct CommandTraits<core::OneAxleSteeringCommand>
{
  using Measure = core::OneAxleSteeringMeasure;
  using MeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
  using CommandLimits = core::OneAxleSteeringCommandLimits;
};

template<>
struct CommandTraits<core::TwoAxleSteeringCommand>
{
  using Measure = core::TwoAxleSteeringMeasure;
  using MeasureMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringMeasureStamped;
  using CommandLimits = core::TwoAxleSteeringCommandLimits;
};

template<>
struct CommandTraits<core::SkidSteeringCommand>
{
  using Measure = core::SkidSteeringMeasure;
  using MeasureMsg = romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped;
  using CommandLimits = core::SkidSteeringCommandLimits;
};

template<>
struct CommandTraits<core::OmniSteeringCommand>
{
  using Measure = core::OmniSteeringMeasure;
  using MeasureMsg = romea_mobile_base_msgs::msg::OmniSteeringMeasureStamped;
  using CommandLimits = core::OmniSteeringCommandLimits;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_TRAITS_HPP_
