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


#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_HPP_

// std
#include <optional>

// ros
#include "rclcpp/time.hpp"

// romea core
#include "romea_core_mobile_base/kinematic/KinematicMeasure.hpp"


namespace romea
{
namespace ros2
{

class DeadReckoning
{
public:
  DeadReckoning();

  void update(const rclcpp::Time & time, const core::KinematicMeasure & kinematic_measure);

  const double & getX()const;

  const double & getY()const;

  const double & getHeading()const;

  void reset();

private:
  std::optional<rclcpp::Time> previous_update_time_;

  double x_;
  double y_;
  double heading_;
  double previous_longitudinal_speed_;
  double previous_lateral_speed_;
  double previous_angular_speed_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_HPP_
