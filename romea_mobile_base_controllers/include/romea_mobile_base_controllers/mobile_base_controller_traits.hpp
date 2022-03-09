#ifndef _romea_MobileBaseControllerTraits_hpp_
#define _romea_MobileBaseControllerTraits_hpp_

#include <romea_core_mobile_base/kinematic/skid_steering/InverseSkidSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/InverseMecanumWheelSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/wheel_steering/InverseFourWheelSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/wheel_steering/InverseTwoWheelSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/InverseTwoAxleSteeringKinematic.hpp>

#include <romea_core_mobile_base/kinematic/skid_steering/ForwardSkidSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/omni_steering/ForwardMecanumWheelSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/wheel_steering/FowardFourWheelSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/wheel_steering/FowardTwoWheelSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/axle_steering/FowardTwoAxleSteeringKinematic.hpp>

#include <romea_mobile_base_msgs/msg/kinematic_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/one_axle_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/two_axle_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/skid_steering_measure_stamped.hpp>
#include <romea_mobile_base_msgs/msg/omni_steering_measure_stamped.hpp>

#include <romea_mobile_base_msgs/msg/one_axle_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/two_axle_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/skid_steering_command.hpp>
#include <romea_mobile_base_msgs/msg/omni_steering_command.hpp>

#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface1FAS2FWD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface1FAS2RWD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface1FWS2RWD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface2AS4WD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface2FWS2RWD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface2FWS2FWD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface2FWS4WD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface2WD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface4WD.hpp"
#include "romea_mobile_base_controller_interfaces/mobile_base_controller_interface4WS4WD.hpp"

#include <four_wheel_steering_msgs/msg/four_wheel_steering.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace romea
{

template <class OdometryFrameType, class KinematicType>
struct MobileBaseControllerTraits
{

};

template <>
struct MobileBaseControllerTraits<OdometryFrame1FAS2RWD,OneAxleSteeringKinematic>
{
    using Kinematic =  OneAxleSteeringKinematic;
    using Command = OneAxleSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
    using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
    using CommandLimits = OneAxleSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface1FAS2RWD;
    using OdometryMeasure = OneAxleSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
};

template <>
struct MobileBaseControllerTraits<OdometryFrame1FAS2FWD,OneAxleSteeringKinematic>
{
    using Kinematic =  OneAxleSteeringKinematic;
    using Command = OneAxleSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
    using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
    using CommandLimits = OneAxleSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface1FAS2FWD;
    using OdometryMeasure = OneAxleSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
};

template <>
struct MobileBaseControllerTraits<OdometryFrame1FWS2RWD,OneAxleSteeringKinematic>
{
    using Kinematic =  OneAxleSteeringKinematic;
    using Command = OneAxleSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
    using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
    using CommandLimits = OneAxleSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface1FWS2RWD;
    using OdometryMeasure = OneAxleSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
};


template <>
struct MobileBaseControllerTraits<OdometryFrame2AS4WD,TwoAxleSteeringKinematic>
{
    using Kinematic =  TwoAxleSteeringKinematic;
    using Command = TwoAxleSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringCommand;
    using CommandRosMsg = four_wheel_steering_msgs::msg::FourWheelSteering;
    using CommandLimits = OneAxleSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface2AS4WD;
    using OdometryMeasure = TwoAxleSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringMeasureStamped;
};

template <>
struct MobileBaseControllerTraits<OdometryFrame2FWS2FWD,TwoWheelSteeringKinematic>
{
    using Kinematic =  TwoWheelSteeringKinematic;
    using Command = OneAxleSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
    using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
    using CommandLimits = OneAxleSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface2FWS2FWD;
    using OdometryMeasure = OneAxleSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
};

template <>
struct MobileBaseControllerTraits<OdometryFrame2FWS2RWD,TwoWheelSteeringKinematic>
{
    using Kinematic =  TwoWheelSteeringKinematic;
    using Command = OneAxleSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
    using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
    using CommandLimits = OneAxleSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface2FWS2RWD;
    using OdometryMeasure = OneAxleSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
};

template <>
struct MobileBaseControllerTraits<OdometryFrame2FWS4WD,TwoWheelSteeringKinematic>
{
    using Kinematic =  TwoWheelSteeringKinematic;
    using Command = OneAxleSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::OneAxleSteeringCommand;
    using CommandRosMsg = ackermann_msgs::msg::AckermannDrive;
    using CommandLimits = OneAxleSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface2FWS4WD;
    using OdometryMeasure = OneAxleSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OneAxleSteeringMeasureStamped;
};

template <>
struct MobileBaseControllerTraits<OdometryFrame4WD,SkidSteeringKinematic>
{
    using Kinematic =  SkidSteeringKinematic;
    using Command = SkidSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::SkidSteeringCommand;
    using CommandRosMsg = geometry_msgs::msg::Twist;
    using CommandLimits = SkidSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface4WD;
    using OdometryMeasure = SkidSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped;
};

template <>
struct MobileBaseControllerTraits<OdometryFrame2WD,SkidSteeringKinematic>
{
    using Kinematic =  SkidSteeringKinematic;
    using Command = SkidSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::SkidSteeringCommand;
    using CommandRosMsg = geometry_msgs::msg::Twist;
    using CommandLimits = SkidSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface2WD;
    using OdometryMeasure = SkidSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::SkidSteeringMeasureStamped;
};


template <>
struct MobileBaseControllerTraits<OdometryFrame4WD,MecanumWheelSteeringKinematic>
{
    using Kinematic = MecanumWheelSteeringKinematic;
    using Command = OmniSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::OmniSteeringCommand;
    using CommandRosMsg = geometry_msgs::msg::Twist;
    using CommandLimits = OmniSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface4WD;
    using OdometryMeasure = OmniSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::OmniSteeringMeasureStamped;
};

template <>
struct MobileBaseControllerTraits<OdometryFrame4WS4WD,FourWheelSteeringKinematic>
{
    using Kinematic =  FourWheelSteeringKinematic;
    using Command = TwoAxleSteeringCommand;
    using CommandMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringCommand;
    using CommandRosMsg = four_wheel_steering_msgs::msg::FourWheelSteering;
    using CommandLimits = TwoAxleSteeringCommandLimits;
    using OdometryInterface = MobileBaseControllerInterface4WS4WD;
    using OdometryMeasure = TwoAxleSteeringMeasure;
    using OdometryMeasureMsg = romea_mobile_base_msgs::msg::TwoAxleSteeringMeasureStamped;
};
}

#endif
