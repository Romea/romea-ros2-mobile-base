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
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_mobile_base_simulation/generic_simulation_system_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
GenericSimulationSystemInterface<HardwareInterface>::GenericSimulationSystemInterface(
  const std::string & hardware_interface_name)
: hardware_interface_name_(hardware_interface_name),
  hardware_interface_(nullptr),
  node_(nullptr),
  joint_state_pub_(nullptr),
  joint_state_sub_(nullptr),
  has_feedback_(false)
{
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface<HardwareInterface>::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (load_info_(hardware_info) == hardware_interface::return_type::OK &&
    load_interface_(hardware_info) == hardware_interface::return_type::OK)
  {
    node_ = std::make_shared<rclcpp::Node>("joint_state_bridge");

    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "bridge/joint_state_command", sensor_data_qos());

    auto callback = std::bind(
      &GenericSimulationSystemInterface<HardwareInterface>::feedback_callback_,
      this, std::placeholders::_1);

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "bridge/joint_state_feedback", best_effort(1), callback);

    return CallbackReturn::SUCCESS;
  } else {
    return CallbackReturn::ERROR;
  }
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
hardware_interface::return_type GenericSimulationSystemInterface<HardwareInterface>::load_info_(
  const hardware_interface::HardwareInfo & /*hardware_info*/)
{
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
hardware_interface::return_type GenericSimulationSystemInterface<HardwareInterface>::load_interface_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    hardware_interface_ = std::make_unique<HardwareInterface>(
      hardware_info,
      hardware_interface::HW_IF_VELOCITY);
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("GenericSimulationSystemInterface"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface<HardwareInterface>::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_configure : previous state " << int(previous_state.id()) << " " << previous_state.label());

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface<HardwareInterface>::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_cleanup : previous state " << int(previous_state.id()) << " " << previous_state.label());

  return CallbackReturn::SUCCESS;
}


//-----------------------------------------------------------------------------
template<typename HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface<HardwareInterface>::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_activate : previous state " << int(previous_state.id()) << " " << previous_state.label());

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface<HardwareInterface>::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_deactivate : previous state" << int(previous_state.id()) << " " << previous_state.label());

  return CallbackReturn::SUCCESS;
}


//-----------------------------------------------------------------------------
template<typename HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface<HardwareInterface>::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_shutdownn : previous state " << int(previous_state.id()) << " " << previous_state.label());

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GenericSimulationSystemInterface<HardwareInterface>::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger(hardware_interface_name_),
    "on_error : previous state " << int(previous_state.id()) << " " << previous_state.label());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type GenericSimulationSystemInterface<HardwareInterface>::read()
#else
hardware_interface::return_type GenericSimulationSystemInterface<HardwareInterface>::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
#endif
{
  rclcpp::spin_some(node_);
  std::lock_guard<std::mutex> guard(mutex_);
  if (has_feedback_) {
    hardware_interface_->set_feedback(feedback_);
    has_feedback_ = false;
  }
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type GenericSimulationSystemInterface<HardwareInterface>::write()
#else
hardware_interface::return_type GenericSimulationSystemInterface<HardwareInterface>::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
#endif
{
  command_ = hardware_interface_->get_joint_state_command();
  // command_.header.frame_id = ""
  joint_state_pub_->publish(command_);
  return hardware_interface::return_type::OK;
}


//-----------------------------------------------------------------------------
template<typename HardwareInterface>
void GenericSimulationSystemInterface<HardwareInterface>::feedback_callback_(
  sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> guard(mutex_);
  has_feedback_ = true;
  feedback_ = *msg;
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
std::vector<hardware_interface::StateInterface>
GenericSimulationSystemInterface<HardwareInterface>::export_state_interfaces()
{
  return hardware_interface_->export_state_interfaces();
}

//-----------------------------------------------------------------------------
template<typename HardwareInterface>
std::vector<hardware_interface::CommandInterface>
GenericSimulationSystemInterface<HardwareInterface>::export_command_interfaces()
{
  return hardware_interface_->export_command_interfaces();
}

// template class GenericSimulationSystemInterface<HardwareInterface2WD>;
// template class GenericSimulationSystemInterface<HardwareInterface4WD>;
template class GenericSimulationSystemInterface<HardwareInterface4WS4WD>;
template class GenericSimulationSystemInterface<HardwareInterface2FWS4WD>;
// template class GenericSimulationSystemInterface<HardwareInterface2FWS2RWD>;
// template class GenericSimulationSystemInterface<HardwareInterface2FWS2FWD>;
// template class GenericSimulationSystemInterface<HardwareInterface2AS4WD>;
// template class GenericSimulationSystemInterface<HardwareInterface2AS2FWD>;
// template class GenericSimulationSystemInterface<HardwareInterface2AS2RWD>;
// template class GenericSimulationSystemInterface<HardwareInterface1FAS2FWD>;
// template class GenericSimulationSystemInterface<HardwareInterface1FAS2RWD>;
// template class GenericSimulationSystemInterface<HardwareInterface1FAS4WD>;
// template class GenericSimulationSystemInterface<HardwareInterface2TD>;
// template class GenericSimulationSystemInterface<HardwareInterface2THD>;
// template class GenericSimulationSystemInterface<HardwareInterface2TTD>;

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GenericSimulationSystemInterface2FWS4WD,
  hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GenericSimulationSystemInterface4WS4WD,
  hardware_interface::SystemInterface)
