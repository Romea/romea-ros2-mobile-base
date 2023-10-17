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


#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_ENHANCED_CONTROLLER_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_ENHANCED_CONTROLLER_HPP_

// std
#include <memory>

// romea
#include "romea_core_common/control/PID.hpp"
#include "romea_core_common/signal/FirstOrderButterworth.hpp"
#include "romea_mobile_base_controllers/mobile_base_controller.hpp"

// ros
#include "sensor_msgs/msg/imu.hpp"


namespace romea
{

template<typename InterfaceType, typename KinematicType>
class MobileBaseEnhancedController : public MobileBaseController<InterfaceType, KinematicType>
{
public:
  using ImuMsg = sensor_msgs::msg::Imu;
  using AngularSpeedPID = PID;
  using AngularSpeedFilter = FirstOrderButterworth;

public:
  MobileBaseEnhancedController();

  virtual ~MobileBaseEnhancedController() = default;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

protected:
  void init_imu_subscriber_();

  void init_angular_speed_pid_();

  void init_angular_speed_filter_();

  void imu_callback_(ImuMsg::ConstSharedPtr msg);

protected:
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;

  std::unique_ptr<AngularSpeedPID> angular_speed_pid_;
  std::unique_ptr<AngularSpeedFilter> angular_speed_filter_;
  std::atomic<double> angular_speed_measure_;
};

using MobileBaseEnhancedController4WD =
  MobileBaseEnhancedController<ControllerInterface4WD, SkidSteeringKinematic>;

using MobileBaseEnhancedController2TD =
  MobileBaseEnhancedController<ControllerInterface2TD, SkidSteeringKinematic>;

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_ENHANCED_CONTROLLER_HPP_
