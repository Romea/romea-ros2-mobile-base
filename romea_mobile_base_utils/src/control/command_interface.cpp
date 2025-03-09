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
#include <memory>
#include <string>

// romea
#include "romea_core_common/time/Time.hpp"

// local
#include "romea_mobile_base_utils/control/command_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename CommandType>
CommandInterface<CommandType>::~CommandInterface()
{
  // try {
  //   cmd_mux_client_.unsubscribe(cmd_pub_->get_topic_name());
  // } catch (const std::exception & e) {
  //   RCLCPP_ERROR_STREAM(logger_, e.what());
  // }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::subscribe_to_cmd_mux(
  const int & priority,
  const double & timeout)
{
  if (priority != -1) {
    try {
      cmd_mux_client_.subscribe(cmd_pub_->get_topic_name(), priority, timeout);
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(logger_, e.what());
    }
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::unsubscribe_to_cmd_mux()
{
  try {
    cmd_mux_client_.unsubscribe(cmd_pub_->get_topic_name());
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM(logger_, e.what());
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
bool CommandInterface<CommandType>::is_started()
{
  return is_started_;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_started_) {
    cmd_pub_->activate();
    last_command_date_ = clock_->now();
    is_started_ = true;
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::stop(bool reset)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (reset) {
    publish_command_(true);
  }
  is_started_ = false;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::enable_emergency_stop()
{
  is_emergency_stop_activated_ = true;
  if (!is_started()) {
    start();
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::disable_emergency_stop()
{
  stop(false);
  is_emergency_stop_activated_ = false;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::connect_timeout_callback(
  std::function<void(void)> callback)
{
  timeout_callback_ = callback;
}


//-----------------------------------------------------------------------------
template<typename CommandType>
bool CommandInterface<CommandType>::is_emergency_stop_activated()
{
  return is_emergency_stop_activated_;
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::send_command(const CommandType & command)
{
  if (!is_emergency_stop_activated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_date_ = clock_->now();
    command_ = command;
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::send_null_command()
{
  if (!is_emergency_stop_activated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_date_ = clock_->now();
    command_ = CommandType();
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::publish_command_(const bool & timeout)
{
  //  //send command msg
  //  typename VehicleControlTraits<CommandType>::CommandMsg msg;
  //  if(!timeout)
  //  {
  //    to_ros_msg(command_,msg);
  //  }

  //  //  std::cout <<" command msg " <<std::endl;
  //  //  std::cout << msg << std::endl;
  //  commandPublisher_.publish(msg);


  if (is_started()) {
    if (!timeout) {
      cmd_pub_->publish(command_);
    } else {
      cmd_pub_->publish(CommandType());
    }
  }
}

//-----------------------------------------------------------------------------
template<typename CommandType>
void CommandInterface<CommandType>::timer_callback_()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (is_emergency_stop_activated()) {
    publish_command_(true);
  } else {
    bool timeout = clock_->now() > last_command_date_ + timeout_duration_;
    if (timeout && timeout_callback_) {
      cmd_pub_->deactivate();
      RCLCPP_INFO_STREAM(logger_, "Publisher timeout");
      timeout_callback_();
    }

    publish_command_(timeout);
  }
}

template class CommandInterface<core::SkidSteeringCommand>;
template class CommandInterface<core::OmniSteeringCommand>;
template class CommandInterface<core::OneAxleSteeringCommand>;
template class CommandInterface<core::TwoAxleSteeringCommand>;


}  // namespace  ros2
}  // namespace  romea
