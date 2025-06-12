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


#ifndef ROMEA_TELEOP_DRIVERS__COMMAND_PARAMETERS_HPP_
#define ROMEA_TELEOP_DRIVERS__COMMAND_PARAMETERS_HPP_


// std
#include <memory>
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"

namespace romea
{
namespace ros2
{

struct MaximalSpeeds
{
  double slow_mode;
  double turbo_mode;
};

void declare_maximal_steering_angle(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_front_steering_angle(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_rear_steering_angle(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_linear_speeds(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_lateral_speeds(std::shared_ptr<rclcpp::Node> node);

void declare_maximal_angular_speeds(std::shared_ptr<rclcpp::Node> node);


double get_maximal_steering_angle(std::shared_ptr<rclcpp::Node> node);

double get_maximal_front_steering_angle(std::shared_ptr<rclcpp::Node> node);

double get_maximal_rear_steering_angle(std::shared_ptr<rclcpp::Node> node);

MaximalSpeeds get_maximal_linear_speeds(std::shared_ptr<rclcpp::Node> node);

MaximalSpeeds get_maximal_lateral_speeds(std::shared_ptr<rclcpp::Node> node);

MaximalSpeeds get_maximal_angular_speeds(std::shared_ptr<rclcpp::Node> node);


void declare_command_output_message_type(std::shared_ptr<rclcpp::Node> node);

std::string get_command_output_message_type(std::shared_ptr<rclcpp::Node> node);

void declare_command_output_message_priority(std::shared_ptr<rclcpp::Node> node);

int get_command_output_message_priority(std::shared_ptr<rclcpp::Node> node);


}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_TELEOP_DRIVERS__COMMAND_PARAMETERS_HPP_
