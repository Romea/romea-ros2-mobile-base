#ifndef _romea_CommandInterface_hpp_
#define _romea_CommandInterface_hpp_

//std
#include <atomic>
#include <functional>
#include <string>
#include <mutex>

//romea
#include "../conversions/kinematic_conversions.hpp"
#include <romea_cmd_mux_utils/cmd_mux_client.hpp>
#include "command_publisher.hpp"

namespace romea {

struct CommandInterfaceConfiguration
{
  std::string output_message_type;
  int priority;
  double rate;
};


template <typename CommandType>
class CommandInterface
{
public :

  using CmdPubType = CommandPublisher<CommandType>;

public :

  CommandInterface(std::shared_ptr<rclcpp::Node> node,
                   CommandInterfaceConfiguration & configuration);

  ~CommandInterface();

  void send_null_command();

  void send_command(const CommandType & command);

  void connect_timeout_callback( std::function<void(void)> callback);

public :

  void start();

  void stop(bool reset);

  void enable_emergency_stop();

  void disable_emergency_stop();

public :

  bool is_started();

  bool is_emergency_stop_activated();

private :

  void timer_callback_();

  void publish_command_(const bool & timeout);

  void subscribe_to_cmd_mux_(const int & priority,
                             const double & timeout);

  void unsubscribe_from_cmd_mux_();

private:

  void create_timer_(std::shared_ptr<rclcpp::Node> node,
                     const double & period);

  void create_publisher_(std::shared_ptr<rclcpp::Node> node,
                         const std::string & output_message_type);

  void create_cmd_mux_client_(std::shared_ptr<rclcpp::Node> node);

private:

  std::unique_ptr<CmdPubType> cmd_pub_;
  std::unique_ptr<CmdMuxClient> cmd_mux_client_;

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

}// namespace 
#endif
