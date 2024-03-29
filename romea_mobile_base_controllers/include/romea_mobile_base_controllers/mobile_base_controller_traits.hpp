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


#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_CONTROLLER_TRAITS_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_CONTROLLER_TRAITS_HPP_

// ros
#include "four_wheel_steering_msgs/msg/four_wheel_steering.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"

// romea core
#include "romea_core_mobile_base/kinematic/skid_steering/InverseSkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/InverseMecanumWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/InverseFourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/InverseTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/InverseOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/InverseTwoAxleSteeringKinematic.hpp"

#include "romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/omni_steering/ForwardMecanumWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardFourWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardOneAxleSteeringKinematic.hpp"
#include "romea_core_mobile_base/kinematic/axle_steering/FowardTwoAxleSteeringKinematic.hpp"

// romea ros
#include "romea_mobile_base_msgs/msg/kinematic_measure_stamped.hpp"
#include "romea_mobile_base_msgs/msg/one_axle_steering_measure_stamped.hpp"
#include "romea_mobile_base_msgs/msg/two_axle_steering_measure_stamped.hpp"
#include "romea_mobile_base_msgs/msg/skid_steering_measure_stamped.hpp"
#include "romea_mobile_base_msgs/msg/omni_steering_measure_stamped.hpp"

#include "romea_mobile_base_msgs/msg/one_axle_steering_command.hpp"
#include "romea_mobile_base_msgs/msg/two_axle_steering_command.hpp"
#include "romea_mobile_base_msgs/msg/skid_steering_command.hpp"
#include "romea_mobile_base_msgs/msg/omni_steering_command.hpp"

// local
#include "romea_mobile_base_controllers/interfaces/controller_interface1FAS2FWD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface1FAS2RWD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface2AS4WD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface2FWS2RWD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface2FWS2FWD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface2FWS4WD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface2TD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface2WD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface4WD.hpp"
#include "romea_mobile_base_controllers/interfaces/controller_interface4WS4WD.hpp"


namespace romea
{
namespace ros2
{

template<class ControllerInterface, class KinematicType>
struct MobileBaseControllerTraits
{
};

template<>
struct MobileBaseControllerTraits<ControllerInterface1FAS2RWD, core::OneAxleSteeringKinematic>
{
  using Kinematic = core::OneAxleSteeringKinematic;
  using Command = core::OneAxleSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
  using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
  using CommandLimits = core::OneAxleSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame1FAS2RWD;
  using OdometryMeasure = core::OneAxleSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo1FAS2RWD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface1FAS2FWD, core::OneAxleSteeringKinematic>
{
  using Kinematic = core::OneAxleSteeringKinematic;
  using Command = core::OneAxleSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
  using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
  using CommandLimits = core::OneAxleSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame1FAS2FWD;
  using OdometryMeasure = core::OneAxleSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo1FAS2FWD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface2AS4WD, core::TwoAxleSteeringKinematic>
{
  using Kinematic = core::TwoAxleSteeringKinematic;
  using Command = core::TwoAxleSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringCommand;
  using CommandRosMsg = four_wheel_steering_msgs::msg::FourWheelSteering;
  using CommandLimits = core::TwoAxleSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame2AS4WD;
  using OdometryMeasure = core::TwoAxleSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo2AS4WD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface2FWS2FWD, core::TwoWheelSteeringKinematic>
{
  using Kinematic = core::TwoWheelSteeringKinematic;
  using Command = core::OneAxleSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
  using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
  using CommandLimits = core::OneAxleSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame2FWS2FWD;
  using OdometryMeasure = core::OneAxleSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo2FWS2FWD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface2FWS2RWD, core::TwoWheelSteeringKinematic>
{
  using Kinematic = core::TwoWheelSteeringKinematic;
  using Command = core::OneAxleSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
  using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
  using CommandLimits = core::OneAxleSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame2FWS2RWD;
  using OdometryMeasure = core::OneAxleSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo2FWS2RWD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface2FWS4WD, core::TwoWheelSteeringKinematic>
{
  using Kinematic = core::TwoWheelSteeringKinematic;
  using Command = core::OneAxleSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
  using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
  using CommandLimits = core::OneAxleSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame2FWS4WD;
  using OdometryMeasure = core::OneAxleSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo2FWS4WD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface4WD, core::SkidSteeringKinematic>
{
  using Kinematic = core::SkidSteeringKinematic;
  using Command = core::SkidSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::SkidSteeringCommand;
  using CommandRosMsg = geometry_msgs::msg::Twist;
  using CommandLimits = core::SkidSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame4WD;
  using OdometryMeasure = core::SkidSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo4WD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface2WD, core::SkidSteeringKinematic>
{
  using Kinematic = core::SkidSteeringKinematic;
  using Command = core::SkidSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::SkidSteeringCommand;
  using CommandRosMsg = geometry_msgs::msg::Twist;
  using CommandLimits = core::SkidSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame2WD;
  using OdometryMeasure = core::SkidSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo2WD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface2TD, core::SkidSteeringKinematic>
{
  using Kinematic = core::SkidSteeringKinematic;
  using Command = core::SkidSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::SkidSteeringCommand;
  using CommandRosMsg = geometry_msgs::msg::Twist;
  using CommandLimits = core::SkidSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame2TD;
  using OdometryMeasure = core::SkidSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo2TD;
};


template<>
struct MobileBaseControllerTraits<ControllerInterface4WD, core::MecanumWheelSteeringKinematic>
{
  using Kinematic = core::MecanumWheelSteeringKinematic;
  using Command = core::OmniSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::OmniSteeringCommand;
  using CommandRosMsg = geometry_msgs::msg::Twist;
  using CommandLimits = core::OmniSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame4WD;
  using OdometryMeasure = core::OmniSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OmniSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo4WD;
};

template<>
struct MobileBaseControllerTraits<ControllerInterface4WS4WD, core::FourWheelSteeringKinematic>
{
  using Kinematic = core::FourWheelSteeringKinematic;
  using Command = core::TwoAxleSteeringCommand;
  using CommandMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringCommand;
  using CommandRosMsg = four_wheel_steering_msgs::msg::FourWheelSteering;
  using CommandLimits = core::TwoAxleSteeringCommandLimits;
  using OdometryFrame = core::OdometryFrame4WS4WD;
  using OdometryMeasure = core::TwoAxleSteeringMeasure;
  using OdometryMeasureMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringMeasureStamped;
  using MobileBaseInfo = core::MobileBaseInfo4WS4WD;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_CONTROLLER_TRAITS_HPP_
