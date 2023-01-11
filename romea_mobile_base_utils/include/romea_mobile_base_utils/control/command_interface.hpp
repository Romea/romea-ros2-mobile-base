// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_

// romea
#include <romea_cmd_mux_utils/cmd_mux_interface.hpp>

// std
#include <atomic>
#include <memory>
#include <functional>
#include <string>
#include <mutex>

#include "romea_mobile_base_utils/conversions/kinematic_conversions.hpp"
#include "romea_mobile_base_utils/control/command_publisher.hpp"

namespace romea
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
  CommandInterface(
    std::shared_ptr<rclcpp::Node> node,
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

  void create_timer_(
    std::shared_ptr<rclcpp::Node> node,
    const double & period);

  void create_publisher_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & output_message_type);

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

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_UTILS__CONTROL__COMMAND_INTERFACE_HPP_
