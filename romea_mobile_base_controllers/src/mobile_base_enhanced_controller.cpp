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
#include <limits>
#include <memory>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_common_utils/params/control_parameters.hpp"
#include "romea_mobile_base_controllers/mobile_base_enhanced_controller.hpp"

namespace
{
const char ANGULAR_SPEED_PID_PARAM_NAME[] = "controller.angular_speed.pid";
const char ANGULAR_SPEED_FILTER_ALPHA_PARAM_NAME[] = "controller.angular_speed.filter.alpha";
}

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
MobileBaseEnhancedController<InterfaceType, KinematicType>::MobileBaseEnhancedController()
: MobileBaseController<InterfaceType, KinematicType>::MobileBaseController(),
    imu_sub_(nullptr),
  angular_speed_pid_(nullptr),
  angular_speed_filter_(nullptr),
  angular_speed_measure_(std::numeric_limits<double>::quiet_NaN())
{
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
CallbackReturn MobileBaseEnhancedController<InterfaceType, KinematicType>::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
//  std::cout << " on configure" << std::endl;

  try {
    this->load_command_limits_();
    this->load_publish_period_();
    this->load_command_timeout_();
    this->load_joints_names_();
    this->init_interface_();
    this->init_angular_speed_pid_();
    this->init_angular_speed_filter_();
    this->init_publishers_();
    this->init_imu_subscriber_();
    this->init_cmd_subscriber_();
    //    std::cout << " on configure OK" << std::endl;
    return CallbackReturn::SUCCESS;
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), e.what());
    return CallbackReturn::ERROR;
  }
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
controller_interface::return_type
MobileBaseEnhancedController<InterfaceType, KinematicType>::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  this->update_time_ = this->get_node()->get_clock()->now();  // why not time?
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "update_controller_state_");
  this->update_controller_state_();
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "publish_controller_state_");
  this->publish_controller_state_();
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "read command");

  auto current_command = this->command_buffer_.consume();
  // std::cout << " update " << update_time_.seconds() << " " << period.seconds() << std::endl;
  if (current_command.has_value()) {
    this->current_command_ = *current_command;
    //    RCLCPP_INFO_STREAM(this->get_node()->get_logger(),"odometry frame measured");
    //    RCLCPP_INFO_STREAM(this->get_node()->get_logger(),"\n"<<odometry_frame_);

    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), " new command ok");
    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "\n" << this->current_command_.cmd);

    RCLCPP_INFO_STREAM(
      this->get_node()->get_logger(), " angular_speed measure " << this->angular_speed_measure_);

    if (isfinite(this->angular_speed_measure_)) {
      this->current_command_.cmd.angularSpeed = this->angular_speed_pid_->compute(
        to_romea_duration(this->current_command_.stamp),
        this->current_command_.cmd.angularSpeed,
        this->angular_speed_measure_);
      RCLCPP_INFO_STREAM(this->get_node()->get_logger(), " new angular speed command");
      RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "\n" << this->current_command_.cmd);
    } else {
      RCLCPP_INFO_STREAM(
        this->get_node()->get_logger(), " no angular speed provided, check imu input");
    }

    this->clamp_current_command_();
    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), " new clamp command");
    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "\n" << this->current_command_.cmd);

    this->send_current_command_();
//    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "cooucou new command");

  } else if (this->timeout_()) {
    //    RCLCPP_INFO_STREAM(this->get_node()->get_logger(), "timeout, brake");
    this->send_null_command();
  }

  return controller_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
void MobileBaseEnhancedController<InterfaceType, KinematicType>::init_imu_subscriber_()
{
  auto callback = std::bind(
    &MobileBaseEnhancedController<InterfaceType, KinematicType>::imu_callback_,
    this, std::placeholders::_1);

  this->imu_sub_ = this->get_node()->template create_subscription<ImuMsg>(
    "imu/data", best_effort(1), callback);
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
void MobileBaseEnhancedController<InterfaceType, KinematicType>::init_angular_speed_pid_()
{
  this->angular_speed_pid_ = std::make_unique<AngularSpeedPID>(
    get_pid_parameters(this->get_node(), ANGULAR_SPEED_PID_PARAM_NAME));
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
void MobileBaseEnhancedController<InterfaceType, KinematicType>::init_angular_speed_filter_()
{
  this->angular_speed_filter_ = std::make_unique<AngularSpeedFilter>(
    get_parameter<double>(this->get_node(), ANGULAR_SPEED_FILTER_ALPHA_PARAM_NAME));
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
void MobileBaseEnhancedController<InterfaceType, KinematicType>::imu_callback_(
  ImuMsg::ConstSharedPtr msg)
{
  auto filtered_angular_speed = this->angular_speed_filter_->update(msg->angular_velocity.z);
  this->angular_speed_measure_.store(filtered_angular_speed);
}

template class MobileBaseEnhancedController<ControllerInterface4WD, core::SkidSteeringKinematic>;
template class MobileBaseEnhancedController<ControllerInterface2TD, core::SkidSteeringKinematic>;

}  // namespace ros2
}  // namespace romea

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  romea::ros2::MobileBaseEnhancedController4WD,
  controller_interface::ControllerInterface)

CLASS_LOADER_REGISTER_CLASS(
  romea::ros2::MobileBaseEnhancedController2TD,
  controller_interface::ControllerInterface)
