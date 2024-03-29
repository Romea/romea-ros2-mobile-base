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


#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_CONTROLLER_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_CONTROLLER_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"

// romea
#include "romea_core_common/concurrency/SharedOptionalVariable.hpp"
#include "romea_common_utils/realtime_publishers/stamped_data_publisher.hpp"

// local
#include "dead_reckoning.hpp"
#include "dead_reckoning_publisher.hpp"
#include "mobile_base_controller_traits.hpp"


namespace romea
{
namespace ros2
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template<typename InterfaceType, typename KinematicType>
class MobileBaseController : public controller_interface::ControllerInterface
{
  using Kinematic =
    typename MobileBaseControllerTraits<InterfaceType, KinematicType>::Kinematic;
  using MobileBaseInfo =
    typename MobileBaseControllerTraits<InterfaceType, KinematicType>::MobileBaseInfo;
  using Command =
    typename MobileBaseControllerTraits<InterfaceType, KinematicType>::Command;
  using CommandMsg =
    typename MobileBaseControllerTraits<InterfaceType, KinematicType>::CommandMsg;
  using CommandRosMsg =
    typename MobileBaseControllerTraits<InterfaceType, KinematicType>::CommandRosMsg;
  using CommandLimits =
    typename MobileBaseControllerTraits<InterfaceType, KinematicType>::CommandLimits;
  using OdometryFrame =
    typename  MobileBaseControllerTraits<InterfaceType, KinematicType>::OdometryFrame;
  using OdometryMeasure =
    typename MobileBaseControllerTraits<InterfaceType, KinematicType>::OdometryMeasure;
  using OdometryMeasureMsg =
    typename MobileBaseControllerTraits<InterfaceType, KinematicType>::OdometryMeasureMsg;
  using KinematicMeasure = core::KinematicMeasure;

  using OdometryMeasurePublisher =
    RealtimeStampedMessagePublisher<OdometryMeasure, OdometryMeasureMsg>;
  using KinematicMeasurePublisher =
    RealtimeStampedMessagePublisher<KinematicMeasure,
      romea_mobile_base_msgs::msg::KinematicMeasureStamped>;

  struct StampedCommand
  {
    Command cmd;
    rclcpp::Time stamp;
  };

  using StampedCommandBuffer = core::SharedOptionalVariable<StampedCommand>;

public:
  MobileBaseController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  //  void declare_joints_names_();
  //  void declare_mobile_base_info_();
  //  void declare_command_limits_();
  //  void declare_publish_period_();
  //  void declare_command_timeout_();
  //  void declare_base_frame_id_();
  //  void declare_odom_frame_id_();
  //  void declare_enable_odom_tf_();

  void load_joints_names_();
  MobileBaseInfo load_mobile_base_info_();
  std::string load_base_frame_id_();
  std::string load_odom_frame_id_();
  bool load_enable_odom_tf_();
  void load_command_limits_();
  void load_publish_period_();
  void load_command_timeout_();

  void init_interface_();
  void init_publishers_();
  void init_cmd_subscriber_();


  void reset_();
  void send_null_command();
  bool timeout_();
  void update_controller_state_();
  void publish_controller_state_();
  void clamp_current_command_();
  void send_current_command_();
  void command_callback_(typename CommandMsg::ConstSharedPtr cmd_msg);

protected:
  std::vector<std::string> joints_names_;
  std::unique_ptr<InterfaceType> controller_interface_;
  typename Kinematic::Parameters kinematic_parameters_;
  CommandLimits user_command_limits_;
  OdometryMeasure odometry_measure_;
  KinematicMeasure kinematic_measure_;
  OdometryFrame odometry_frame_;
  std::atomic<bool> is_running_;

  typename rclcpp::Subscription<CommandMsg>::SharedPtr command_sub_;
  StampedCommand previous_command_;
  StampedCommand current_command_;
  StampedCommandBuffer command_buffer_;

  rclcpp::Time update_time_;
  rclcpp::Time last_state_publish_time_;
  rclcpp::Duration publish_period_;
  rclcpp::Duration command_timeout_;

  std::unique_ptr<DeadReckoningPublisher> dead_reckoning_publisher_;
  std::unique_ptr<OdometryMeasurePublisher> odometry_measure_publisher_;
  std::unique_ptr<KinematicMeasurePublisher> kinematic_measure_publisher_;
};


using MobileBaseController1FAS2FWD =
  MobileBaseController<ControllerInterface1FAS2FWD, core::OneAxleSteeringKinematic>;
using MobileBaseController1FAS2RWD =
  MobileBaseController<ControllerInterface1FAS2RWD, core::OneAxleSteeringKinematic>;
// using MobileBaseController1FWS2RWD =
//   MobileBaseController<ControllerInterface1FWS2RWD, core::OneAxleSteeringKinematic>;
using MobileBaseController2AS4WD =
  MobileBaseController<ControllerInterface2AS4WD, core::TwoAxleSteeringKinematic>;
using MobileBaseController2FWS2FWD =
  MobileBaseController<ControllerInterface2FWS2FWD, core::TwoWheelSteeringKinematic>;
using MobileBaseController2FWS2RWD =
  MobileBaseController<ControllerInterface2FWS2RWD, core::TwoWheelSteeringKinematic>;
using MobileBaseController2FWS4WD =
  MobileBaseController<ControllerInterface2FWS4WD, core::TwoWheelSteeringKinematic>;
using MobileBaseController2WD =
  MobileBaseController<ControllerInterface2WD, core::SkidSteeringKinematic>;
using MobileBaseController2TD =
  MobileBaseController<ControllerInterface2TD, core::SkidSteeringKinematic>;
using MobileBaseController4WD =
  MobileBaseController<ControllerInterface4WD, core::SkidSteeringKinematic>;
using MobileBaseController4MWD =
  MobileBaseController<ControllerInterface4WD, core::MecanumWheelSteeringKinematic>;
using MobileBaseController4WS4WD =
  MobileBaseController<ControllerInterface4WS4WD, core::FourWheelSteeringKinematic>;

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_CONTROLLER_HPP_
