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

template<typename CommandType>
struct CommandTraits;

template<>
struct CommandTraits<OneAxleSteeringCommand> {
  using Measure = OneAxleSteeringMeasure;
  using MeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
};

template<>
struct CommandTraits<TwoAxleSteeringCommand> {
  using Measure = TwoAxleSteeringMeasure;
  using MeasureMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringMeasureStamped;
};

template<>
struct CommandTraits<SkidSteeringCommand> {
  using Measure = SkidSteeringMeasure;
  using MeasureMsg = romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped;
};

template<>
struct CommandTraits<OmniSteeringCommand> {
  using Measure = OmniSteeringMeasure;
  using MeasureMsg = romea_mobile_base_msgs::msg::OmniSteeringMeasureStamped;
};

}

#endif  // ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_TRAITS_HPP_
