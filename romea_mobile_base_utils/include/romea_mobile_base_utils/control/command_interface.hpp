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


#ifndef ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_

// std
#include <atomic>
#include <memory>
#include <functional>
#include <string>
#include <mutex>

// romea
#include "romea_cmd_mux_utils/cmd_mux_interface.hpp"

#include "romea_mobile_base_utils/conversions/kinematic_conversions.hpp"
#include "romea_mobile_base_utils/control/command_publisher.hpp"

namespace romea
{
namespace ros2
{

struct CommandInterfaceConfiguration
{
  std::string output_message_type;
  int priority;
  double rate;
};


template<typename CommandType>
class CommandInterface
{
public:
  using CmdPublisher = PublisherBase<CommandType>;
  using Configuration = CommandInterfaceConfiguration;

public:
  template<typename Node>
  CommandInterface(
    std::shared_ptr<Node> node,
    const Configuration & configuration);

  void send_null_command();

  void send_command(const CommandType & command);

  void connect_timeout_callback(std::function<void(void)> callback);

public:
  void start();

  void stop(bool reset);

  void enable_emergency_stop();

  void disable_emergency_stop();

public:
  bool is_started();

  bool is_emergency_stop_activated();

private:
  void timer_callback_();

  void publish_command_(const bool & timeout);

  template<typename Node>
  void create_timer_(std::shared_ptr<Node> node, const double & period);

  template<typename Node>
  void create_publisher_(std::shared_ptr<Node> node, const std::string & output_message_type);

  void subscribe_to_cmd_mux(
    const int & priority,
    const double & timetout);

private:
  std::shared_ptr<CmdPublisher> cmd_pub_;
  CmdMuxInterface cmd_mux_client_;

  std::atomic<bool> is_started_;
  std::atomic<bool> is_emergency_stop_activated_;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Duration timeout_duration_;
  rclcpp::Time last_command_date_;
  CommandType command_;

  std::mutex mutex_;

  std::function<void(void)> timeout_callback_;
};


//-----------------------------------------------------------------------------
template<typename CommandType>
template<typename Node>
CommandInterface<CommandType>::CommandInterface(
  std::shared_ptr<Node> node,
  const Configuration & configuration)
: cmd_pub_(nullptr),
  cmd_mux_client_(node),
  is_started_(false),
  is_emergency_stop_activated_(false),
  clock_(node->get_clock()),
  timer_(),
  timeout_duration_(0, 0),
  last_command_date_(node->now()),
  command_(),
  mutex_(),
  timeout_callback_(nullptr)
{
  const std::string & output_message_type = configuration.output_message_type;
  const int & priority = configuration.priority;
  double period = 1 / configuration.rate;
  double timeout = 2 * period;

  timeout_duration_ = core::durationFromSecond(timeout);
  create_publisher_(node, output_message_type);
  subscribe_to_cmd_mux(priority, timeout);
  create_timer_(node, period);
}

//-----------------------------------------------------------------------------
template<typename CommandType>
template<typename Node>
void CommandInterface<CommandType>::create_publisher_(
  std::shared_ptr<Node> node,
  const std::string & output_message_type)
{
  cmd_pub_ = make_command_publisher<CommandType>(node, output_message_type);
}

//-----------------------------------------------------------------------------
template<typename CommandType>
template<typename Node>
void CommandInterface<CommandType>::create_timer_(
  std::shared_ptr<Node> node,
  const double & period)
{
  auto timer_callback = std::bind(&CommandInterface::timer_callback_, this);
  timer_ = node->create_wall_timer(core::durationFromSecond(period), timer_callback);
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_
