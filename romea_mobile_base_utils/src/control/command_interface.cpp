#include "romea_mobile_base_utils/control/command_interface.hpp"
#include <romea_core_common/time/Time.hpp>
//#include "romea_vehicle_control_utils/vehicle_control_traits.hpp"

namespace romea {

//-----------------------------------------------------------------------------
template <typename CommandType>
CommandInterface<CommandType>::CommandInterface(std::shared_ptr<rclcpp::Node> node,
                                                const Configuration & configuration):
  cmd_pub_(nullptr),
  cmd_mux_client_(node),
  is_started_(false),
  is_emergency_stop_activated_(false),
  clock_(node->get_clock()),
  timer_(),
  timeout_duration_(0,0),
  last_command_date_(),
  command_(),
  mutex_(),
  timeout_callback_(nullptr)
{
  const std::string & output_message_type = configuration.output_message_type;
  const int & priority =configuration.priority;
  double period = 1/configuration.rate;
  double timeout = 2*period;

  timeout_duration_=durationFromSecond(timeout);
  create_publisher_(node,output_message_type);
  subscribe_to_cmd_mux(priority,timeout);
  create_timer_(node,period);
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::create_publisher_(std::shared_ptr<rclcpp::Node> node,
                                                      const std::string & output_message_type)
{
  cmd_pub_ = make_command_publisher<CommandType>(node,output_message_type);
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::create_timer_(std::shared_ptr<rclcpp::Node> node,
                                                  const double & period)
{
  auto timer_callback= std::bind(&CommandInterface::timer_callback_, this);
  timer_ = node->create_wall_timer(durationFromSecond(period),timer_callback);
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::subscribe_to_cmd_mux(const int & priority,
                                                         const double & timeout)
{
  if(priority!=-1)
  {
    cmd_mux_client_.subscribe(cmd_pub_->get_topic_name(),priority,timeout);
  }
}

//-----------------------------------------------------------------------------
template <typename CommandType>
bool CommandInterface<CommandType>::is_started()
{
  return is_started_;
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(!is_started_)
  {
    last_command_date_ = clock_->now();
    is_started_=true;
  }
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::stop(bool reset)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if(reset)
  {
    publish_command_(true);
  }
  is_started_=false;
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::enable_emergency_stop()
{
  is_emergency_stop_activated_=true;
  if(!is_started())
  {
    start();
  }
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::disable_emergency_stop()
{
  stop(false);
  is_emergency_stop_activated_=false;
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::connect_timeout_callback( std::function<void(void)> callback)
{
  timeout_callback_=callback;
}


//-----------------------------------------------------------------------------
template <typename CommandType>
bool CommandInterface<CommandType>::is_emergency_stop_activated()
{
  return is_emergency_stop_activated_;
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void  CommandInterface<CommandType>::send_command(const CommandType & command)
{
  if(!is_emergency_stop_activated_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_date_ = clock_->now();
    command_ = command;
  }
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::send_null_command()
{
  if(!is_emergency_stop_activated_)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_date_ = clock_->now();
    command_ = CommandType();
  }
}

//-----------------------------------------------------------------------------
template <typename CommandType>
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


  if(is_started())
  {
    if(!timeout)
    {
      cmd_pub_->publish(command_);
    }
    else
    {
      cmd_pub_->publish(CommandType());
    }
  }
}

//-----------------------------------------------------------------------------
template <typename CommandType>
void CommandInterface<CommandType>::timer_callback_()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if(is_emergency_stop_activated())
  {
    publish_command_(true);
  }
  else
  {

    bool timeout=clock_->now( )> last_command_date_ + timeout_duration_;
    if(timeout && timeout_callback_)
    {
      std::cout << "Command publisher timeout"<<std::endl;
      timeout_callback_();
    }

    publish_command_(timeout);

  }
}

template class CommandInterface<SkidSteeringCommand>;
template class CommandInterface<OmniSteeringCommand>;
template class CommandInterface<OneAxleSteeringCommand>;
template class CommandInterface<TwoAxleSteeringCommand>;


}// namespace 
