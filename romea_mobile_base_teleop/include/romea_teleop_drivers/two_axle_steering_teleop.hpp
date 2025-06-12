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


#ifndef ROMEA_TELEOP_DRIVERS__TWO_AXLE_STEERING_TELEOP_HPP_
#define ROMEA_TELEOP_DRIVERS__TWO_AXLE_STEERING_TELEOP_HPP_

// std
#include <map>
#include <string>

// romea
#include "romea_teleop_drivers/teleop_base.hpp"

namespace romea
{
namespace ros2
{

class TwoAxleSteeringTeleop : public TeleopBase<core::TwoAxleSteeringCommand>
{
public:
  ROMEA_TELEOP_DRIVERS_PUBLIC
  explicit TwoAxleSteeringTeleop(const rclcpp::NodeOptions & options);

  ROMEA_TELEOP_DRIVERS_PUBLIC
  virtual ~TwoAxleSteeringTeleop() = default;

private:
  void declare_joystick_axes_mapping_() override;

  void declare_joystick_buttons_mapping_() override;

  std::map<std::string, int> get_joystick_axes_mapping_() override;

  std::map<std::string, int> get_joystick_buttons_mapping_() override;

  void  declare_command_ranges_() override;

  void get_command_ranges_() override;

  void joystick_callback_(const Joystick & joy)override;

  double compute_linear_speed_(const double & maximal_linear_speed) const;

  double compute_front_steering_angle_() const;

  double compute_rear_steering_angle_() const;

  void init_axes_control_modes_();

private:
  MaximalSpeeds maximal_linear_speeds_;
  double maximal_front_steering_angle_;
  double maximal_rear_steering_angle_;
  bool two_axes_linear_speed_control_;
  bool two_axes_steering_angle_control_;
  bool sent_disable_msg_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_TELEOP_DRIVERS__TWO_AXLE_STEERING_TELEOP_HPP_
