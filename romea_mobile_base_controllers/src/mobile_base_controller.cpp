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
#include "romea_mobile_base_utils/conversions/command_conversions.hpp"
#include "romea_mobile_base_utils/conversions/kinematic_conversions.hpp"
#include "romea_mobile_base_utils/params/command_limits_parameters.hpp"
#include "romea_mobile_base_utils/params/mobile_base_parameters.hpp"
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/qos.hpp"

// local
#include "romea_mobile_base_controllers/mobile_base_controller.hpp"

namespace
{
const char BASE_FRAME_ID_PARAM_NAME[] = "controller.base_frame_id";
const char ODOM_FRAME_ID_PARAM_NAME[] = "controller.odom_frame_id";
const char ENABLE_ODOM_TF_PARAM_NAME[] = "controller.enable_odom_tf";
const char PUBLISH_RATE_PARAM_NAME[] = "controller.publish_rate";
const char TIMEOUT_PARAM_NAME[] = "controller.timeout";
const char COMMMAND_LIMITS_PARAM_NAME[] = "controller.command_limits";
const char JOINTS_PREFIX_PARAM_NAME[] = "controller.joints_prefix";
const char JOINTS_MAPPING_PARAM_NAME[] = "base_info.joints";
const char MOBILE_BASE_INFO_PARAM_NAME[] = "base_info";

const char DEFAULT_BASE_FRAME_ID[] = "base_link";
const char DEFAULT_ODOM_FRAME_ID[] = "odom";
const double DEFAULT_COMMAND_TIMEOUT = 0.5;
const double DEFAULT_PUBLISH_RATE = 50.0;
}  // namespace

namespace romea
{

template<typename OdometryFrameType, typename KinematicType>
MobileBaseController<OdometryFrameType, KinematicType>::MobileBaseController()
: ControllerInterface(),
  controller_interface_(nullptr),
  kinematic_parameters_(),
  user_command_limits_(),
  odometry_measure_(),
  kinematic_measure_(),
  odometry_frame_(),
  command_sub_(),
  previous_command_(),
  current_command_(),
  command_buffer_(),
  update_time_(),
  last_state_publish_time_(),
  publish_period_(0, 0),
  command_timeout_(0, 0),
  dead_reckoning_publisher_(),
  odometry_measure_publisher_(),
  kinematic_measure_publisher_()
{
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
CallbackReturn MobileBaseController<InterfaceType, KinematicType>::on_init()
{
//  std::cout << " on init" << std::endl;
  try {
    //    declare_command_limits_();
    //    declare_publish_period_();
    //    declare_command_timeout_();
    //    declare_base_frame_id_();
    //    declare_odom_frame_id_();
    //    declare_enable_odom_tf_();
    //    declare_mobile_base_info_();
    //    declare_joints_names_();
    // std::cout << " on init OK" << std::endl;
    return CallbackReturn::SUCCESS;
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), e.what());
    return CallbackReturn::ERROR;
  }
}


//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
controller_interface::InterfaceConfiguration
MobileBaseController<InterfaceType, KinematicType>::command_interface_configuration() const
{
//  std::cout << " command_interface_configuration" << std::endl;
  if (controller_interface_) {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
      InterfaceType::hardware_interface_names(joints_names_)};
  } else {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  }
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
controller_interface::InterfaceConfiguration
MobileBaseController<InterfaceType, KinematicType>::state_interface_configuration() const
{
//  std::cout << " state_interface_configuration" << std::endl;

  if (controller_interface_) {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
      InterfaceType::hardware_interface_names(joints_names_)};
  } else {
    return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  }
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
CallbackReturn MobileBaseController<InterfaceType, KinematicType>::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
//  std::cout << " on configure" << std::endl;

  try {
    load_command_limits_();
    load_publish_period_();
    load_command_timeout_();
    load_joints_names_();
    init_interface_();
    init_publishers_();
    init_cmd_subscriber_();
//    std::cout << " on configure OK" << std::endl;
    return CallbackReturn::SUCCESS;
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), e.what());
    return CallbackReturn::ERROR;
  }
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
CallbackReturn MobileBaseController<InterfaceType, KinematicType>::on_activate(
  const rclcpp_lifecycle::State &)
{
//  std::cout << " on activate" << std::endl;

  try {
//    controller_interface_->register_loaned_command_interfaces(command_interfaces_);
//    controller_interface_->register_loaned_state_interfaces(state_interfaces_);

    auto now = get_node()->get_clock()->now();
    previous_command_.cmd = Command();
    previous_command_.stamp = now;
    current_command_.cmd = Command();
    current_command_.stamp = now;
    last_state_publish_time_ = now;
    send_null_command();

    is_running_ = true;

    return CallbackReturn::SUCCESS;
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), e.what());
    return CallbackReturn::ERROR;
  }
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
CallbackReturn MobileBaseController<InterfaceType, KinematicType>::on_deactivate(
  const rclcpp_lifecycle::State &)
{
//  std::cout << " on deactivate" << std::endl;

  is_running_ = false;
  send_null_command();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
CallbackReturn MobileBaseController<InterfaceType, KinematicType>::on_cleanup(
  const rclcpp_lifecycle::State &)
{
//  std::cout << " on cleanup" << std::endl;

  reset_();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
CallbackReturn MobileBaseController<InterfaceType, KinematicType>::on_error(
  const rclcpp_lifecycle::State &)
{
//  std::cout << " on error" << std::endl;
  reset_();
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
CallbackReturn MobileBaseController<InterfaceType, KinematicType>::on_shutdown(
  const rclcpp_lifecycle::State &)
{
//  std::cout << " on shutdown" << std::endl;
  return CallbackReturn::SUCCESS;
}


//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
controller_interface::return_type MobileBaseController<InterfaceType, KinematicType>::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::cout << "update" << time.seconds() << " " << time.nanoseconds() << std::endl;
  update_time_ = get_node()->get_clock()->now();  // why not time?
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "update_controller_state_");
  update_controller_state_();
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "publish_controller_state_");
  publish_controller_state_();
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "read command");

  auto current_command = command_buffer_.consume();
  // std::cout << " update " << update_time_.seconds() << " " << period.seconds() << std::endl;
  if (current_command.has_value()) {
    current_command_ = *current_command;
    //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"odometry frame measured");
    //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"\n"<<odometry_frame_);

    //    RCLCPP_INFO_STREAM(get_node()->get_logger(), " new command ok");
    //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"\n"<<current_command_.cmd);

    clamp_current_command_();
    //    RCLCPP_INFO_STREAM(get_node()->get_logger(), " new clamp command");
    //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"\n"<<current_command_.cmd);

    send_current_command_();
//    RCLCPP_INFO_STREAM(get_node()->get_logger(), "cooucou new command");

  } else if (timeout_()) {
    //    RCLCPP_INFO_STREAM(get_node()->get_logger(), "timeout, brake");
    send_null_command();
  }

  return controller_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
bool MobileBaseController<OdometryFrameType, KinematicType>::timeout_()
{
  // RCLCPP_INFO_STREAM(
  //   get_node()->get_logger(),
  //   "dts " << update_time_ - current_command_.stamp << " " << command_timeout_);
  // RCLCPP_INFO_STREAM(
  //   get_node()->get_logger(),
  //   "dts " << (update_time_ - current_command_.stamp).seconds() << " " <<
  //     command_timeout_.seconds());
  return update_time_ - current_command_.stamp > command_timeout_;
}

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::publish_controller_state_()
{
  if (last_state_publish_time_ + publish_period_ < update_time_) {
    last_state_publish_time_ += publish_period_;
    dead_reckoning_publisher_->update(update_time_, kinematic_measure_);
    odometry_measure_publisher_->publish(update_time_, odometry_measure_);
    kinematic_measure_publisher_->publish(update_time_, kinematic_measure_);
  }
}

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::update_controller_state_()
{
  controller_interface_->read(state_interfaces_, odometry_frame_);
  //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"odometry frame measured");
  //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"\n"<<odometry_frame_);

  inverseKinematic(kinematic_parameters_, odometry_frame_, odometry_measure_);
  //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"odometry measure");
  //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"\n"<<odometry_measure_);

  kinematic_measure_ = toKinematicMeasure(odometry_measure_, kinematic_parameters_);
  //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"kinmeatic measure");
  //    RCLCPP_INFO_STREAM(get_node()->get_logger(),"\n"<<kinematic_measure_);
}

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::clamp_current_command_()
{
  current_command_.cmd = clamp(
    kinematic_parameters_,
    user_command_limits_,
    current_command_.cmd);


  //    if(kinematic_command_clamp_)
  //    {
  //        double dt = (current_command_.stamp-previous_command_.stamp).toSec();
  //        Command clamped_command = clamp(kinematic_parameters_,
  //                                        previous_command_.cmd,
  //                                        clamped_command_.cmd,
  //                                        dt);
  //   }

  previous_command_ = current_command_;
}

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::send_current_command_()
{
  forwardKinematic(kinematic_parameters_, current_command_.cmd, odometry_frame_);
//  RCLCPP_INFO_STREAM(get_node()->get_logger(),"odometry frame commad");
//  RCLCPP_INFO_STREAM(get_node()->get_logger(),odometry_frame_);

  controller_interface_->write(odometry_frame_, command_interfaces_);
}


//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::send_null_command()
{
  controller_interface_->write(OdometryFrame(), command_interfaces_);
}


//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::command_callback_(
  typename CommandMsg::ConstSharedPtr cmd_msg)
{
//  RCLCPP_INFO_STREAM(get_node()->get_logger(),"command_callback_");
  StampedCommand stamped_cmd;
  to_romea(*cmd_msg, stamped_cmd.cmd);
  stamped_cmd.stamp = get_node()->get_clock()->now();

  if (is_running_) {
    //        ROS_INFO_STREAM("running");
    if (!isValid(stamped_cmd.cmd)) {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "Received NaN in command. Ignoring.");
      return;
    }

    //        ROS_INFO_STREAM("valid");
    //        ROS_INFO_STREAM(stamped_cmd.cmd);
    command_buffer_.store(stamped_cmd);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Can't accept new commands. Controller is not activated.");
  }
}

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::reset_()
{
  command_sub_.reset();
  dead_reckoning_publisher_.reset();

  send_null_command();
  controller_interface_.reset();
}

////-----------------------------------------------------------------------------
// template <typename InterfaceType, typename KinematicType>
// void MobileBaseController<InterfaceType,KinematicType>::declare_joints_names_()
// {
//   declare_parameter_with_default<std::string>(get_node(),JOINTS_PREFIX_PARAM_NAME,"");
//  InterfaceType::declare_joints_names(get_node(),JOINTS_MAPPING_PARAM_NAME);
// }

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
void MobileBaseController<InterfaceType, KinematicType>::load_joints_names_()
{
  std::string prefix = get_parameter_or<std::string>(get_node(), JOINTS_PREFIX_PARAM_NAME, "");
  joints_names_ = InterfaceType::get_joints_names(get_node(), JOINTS_MAPPING_PARAM_NAME);

//  std::cout << " joint_names "<< std::endl;
  for (auto & joint_name : joints_names_) {
    joint_name = prefix + joint_name;
//    std::cout << joint_name << std::endl;
  }
}

////-----------------------------------------------------------------------------
// template <typename InterfaceType, typename KinematicType>
// void MobileBaseController<InterfaceType,KinematicType>::declare_mobile_base_info_()
// {
//  declare_mobile_base_info<MobileBaseInfo>(get_node(),MOBILE_BASE_INFO_PARAM_NAME);
// }

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
typename MobileBaseController<InterfaceType, KinematicType>::MobileBaseInfo
MobileBaseController<InterfaceType, KinematicType>::load_mobile_base_info_()
{
  return get_mobile_base_info<MobileBaseInfo>(get_node(), MOBILE_BASE_INFO_PARAM_NAME);
}

////-----------------------------------------------------------------------------
// template <typename InterfaceType, typename KinematicType>
// void MobileBaseController<InterfaceType,KinematicType>::declare_base_frame_id_()
// {
//  declare_mobile_base_info<std::string>(get_node(),MOBILE_BASE_INFO_PARAM_NAME);
// }

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
std::string MobileBaseController<InterfaceType, KinematicType>::load_base_frame_id_()
{
  auto base_frame_id = get_parameter_or<std::string>(
    get_node(), BASE_FRAME_ID_PARAM_NAME, DEFAULT_BASE_FRAME_ID);

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Base frame_id set to " << base_frame_id);
  return base_frame_id;
}

// //-----------------------------------------------------------------------------
// template <typename InterfaceType, typename KinematicType>
// void MobileBaseController<InterfaceType,KinematicType>::declare_odom_frame_id_()
// {
//   declare_mobile_base_info<std::string>(get_node(),ODOM_FRAME_ID_PARAM_NAME);
// }

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
std::string MobileBaseController<InterfaceType, KinematicType>::load_odom_frame_id_()
{
  auto odom_frame_id = get_parameter_or<std::string>(
    get_node(), ODOM_FRAME_ID_PARAM_NAME, DEFAULT_ODOM_FRAME_ID);

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Odometry frame_id set to " << odom_frame_id);
  return odom_frame_id;
}

////-----------------------------------------------------------------------------
// template <typename InterfaceType, typename KinematicType>
// void MobileBaseController<InterfaceType,KinematicType>::declare_enable_odom_tf_()
// {
//   declare_mobile_base_info<std::string>(get_node(),ENABLE_ODOM_TF_PARAM_NAME);
// }

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
bool MobileBaseController<InterfaceType, KinematicType>::load_enable_odom_tf_()
{
  bool enable_odom_tf = get_parameter_or<bool>(get_node(), ENABLE_ODOM_TF_PARAM_NAME, false);
  RCLCPP_INFO_STREAM(
    get_node()->get_logger(),
    "Publishing to tf is " << (enable_odom_tf ? "enabled" : "disabled"));
  return enable_odom_tf;
}

////-----------------------------------------------------------------------------
// template <typename InterfaceType, typename KinematicType>
// void MobileBaseController<InterfaceType,KinematicType>::declare_publish_period_()
// {
// declare_parameter_with_default<double>(
//   get_node(), PUBLISH_RATE_PARAM_NAME, DEFAULT_PUBLISH_RATE);
// }

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
void MobileBaseController<InterfaceType, KinematicType>::load_publish_period_()
{
  double publish_rate = get_parameter<double>(get_node(), PUBLISH_RATE_PARAM_NAME);
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);

  std::stringstream info_msg;
  info_msg << "Controller state will be published at ";
  info_msg << publish_rate;
  info_msg << "Hz.";
  RCLCPP_INFO_STREAM(get_node()->get_logger(), info_msg.str());
}

////-----------------------------------------------------------------------------
// template <typename InterfaceType, typename KinematicType>
// void MobileBaseController<InterfaceType,KinematicType>::declare_command_timeout_()
// {
//  declare_parameter_with_default<double>(get_node(),TIMEOUT_PARAM_NAME, DEFAULT_COMMAND_TIMEOUT);
// }

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::load_command_timeout_()
{
  double command_timeout = get_parameter<double>(get_node(), TIMEOUT_PARAM_NAME);
  command_timeout_ = rclcpp::Duration::from_seconds(command_timeout);

  std::stringstream info_msg;
  info_msg << "Commands will be considered old if they are older than ";
  info_msg << command_timeout;
  info_msg << "s.";
  RCLCPP_INFO_STREAM(get_node()->get_logger(), info_msg.str());
}

////-----------------------------------------------------------------------------
// template <typename OdometryFrameType, typename KinematicType>
// void MobileBaseController<OdometryFrameType,KinematicType>::declare_command_limits_()
// {
//  declare_command_limits<CommandLimits>(get_node(),COMMMAND_LIMITS_PARAM_NAME);
// }

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::load_command_limits_()
{
  user_command_limits_ = get_command_limits<CommandLimits>(get_node(), COMMMAND_LIMITS_PARAM_NAME);
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
void MobileBaseController<InterfaceType, KinematicType>::init_interface_()
{
//  std::cout << "init_interface_ " << std::endl;
  auto mobile_base_info = load_mobile_base_info_();
  controller_interface_ = std::make_unique<InterfaceType>(mobile_base_info);
  to_kinematic_parameters(mobile_base_info, kinematic_parameters_);
}

//-----------------------------------------------------------------------------
template<typename InterfaceType, typename KinematicType>
void MobileBaseController<InterfaceType, KinematicType>::init_cmd_subscriber_()
{
  std::string cmd_topic = "controller/";
  if (std::is_same<Command, SkidSteeringCommand>::value) {
    cmd_topic += "cmd_skid_steering";
  } else if (std::is_same<Command, OneAxleSteeringCommand>::value) {
    cmd_topic += "cmd_one_axle_steering";
  } else if (std::is_same<Command, TwoAxleSteeringCommand>::value) {
    cmd_topic += "cmd_two_axle_steering";
  } else {
    cmd_topic += "cmd_omni_steering";
  }

  auto callback = std::bind(&MobileBaseController::command_callback_, this, std::placeholders::_1);
  command_sub_ = get_node()->create_subscription<CommandMsg>(cmd_topic, best_effort(1), callback);
}

//-----------------------------------------------------------------------------
template<typename OdometryFrameType, typename KinematicType>
void MobileBaseController<OdometryFrameType, KinematicType>::init_publishers_()
{
  std::string base_frame_id = load_base_frame_id_();
  std::string odom_frame_id = load_odom_frame_id_();
  bool enable_odom_tf = load_enable_odom_tf_();

  odometry_measure_publisher_ = std::make_unique<OdometryMeasurePublisher>(
    get_node(), "controller/odometry", base_frame_id, sensor_data_qos());
  kinematic_measure_publisher_ = std::make_unique<KinematicMeasurePublisher>(
    get_node(), "controller/kinematic", base_frame_id, sensor_data_qos());
  dead_reckoning_publisher_ = std::make_unique<DeadReckoningPublisher>(
    get_node(), odom_frame_id, base_frame_id, enable_odom_tf);
}

template class MobileBaseController<ControllerInterface1FAS2FWD, OneAxleSteeringKinematic>;
template class MobileBaseController<ControllerInterface1FAS2RWD, OneAxleSteeringKinematic>;
// template class MobileBaseController<ControllerInterface1FWS2RWD, OneAxleSteeringKinematic>;
template class MobileBaseController<ControllerInterface2AS4WD, TwoAxleSteeringKinematic>;
template class MobileBaseController<ControllerInterface2FWS2FWD, TwoWheelSteeringKinematic>;
template class MobileBaseController<ControllerInterface2FWS2RWD, TwoWheelSteeringKinematic>;
template class MobileBaseController<ControllerInterface2FWS4WD, TwoWheelSteeringKinematic>;
// template class MobileBaseController<ControllerInterface2TD, SkidSteeringKinematic>;
template class MobileBaseController<ControllerInterface2WD, SkidSteeringKinematic>;
template class MobileBaseController<ControllerInterface4WD, SkidSteeringKinematic>;
template class MobileBaseController<ControllerInterface4WD, MecanumWheelSteeringKinematic>;
template class MobileBaseController<ControllerInterface4WS4WD, FourWheelSteeringKinematic>;

}  // namespace romea

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController1FAS2FWD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController1FAS2RWD,
  controller_interface::ControllerInterface)
// CLASS_LOADER_REGISTER_CLASS(
//  romea::MobileBaseController1FWS2RWD,
//  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController2AS4WD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController2FWS2FWD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController2FWS2RWD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController2FWS4WD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController2TD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController2WD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController4WD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController4MWD,
  controller_interface::ControllerInterface)
CLASS_LOADER_REGISTER_CLASS(
  romea::MobileBaseController4WS4WD,
  controller_interface::ControllerInterface)
