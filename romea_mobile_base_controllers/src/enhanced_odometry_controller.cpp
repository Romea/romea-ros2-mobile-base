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


//  #include "romea_mobile_base_controllers/enhanced_odometry_controller.hpp"


namespace romea
{

// template<typename OdometryFrameType, typename KinematicType>
// EnhancedMobileBaseController<OdometryFrameType, KinematicType>::EnhancedMobileBaseController()
// : MobileBaseController<OdometryFrameType, KinematicType>::MobileBaseController(),
//     angular_speed_pid_(nullptr),
//   angular_speed_measure_()
// {
// }

// //-----------------------------------------------------------------------------
// template<typename OdometryFrameType, typename KinematicType>
// void EnhancedMobileBaseController<OdometryFrameType, KinematicType>::imu_callback_(
//   const sensor_msgs::Imu::ConstPtr & msg)
// {
//   angular_speed_measure_.store(msg->angular_velocity.z);
// }

// //-----------------------------------------------------------------------------
// template<typename OdometryFrameType, typename KinematicType>
// bool EnhancedMobileBaseController<OdometryFrameType, KinematicType>::init(
//   hardware_interface::RobotHW * hw,
//   ros::NodeHandle & root_nh,
//   ros::NodeHandle & controller_nh)
// {
//   this->loadControllerName_(controller_nh);
//   this->loadKinematicParams_(root_nh);
//   this->loadCommandConstraints_(controller_nh);
//   this->loadPublishPeriod_(controller_nh);
//   this->loadCommandTimeout_(controller_nh);
//   this->initPID_(controller_nh);
//   this->initInterface_(hw, root_nh, controller_nh);
//   this->initPublishers_(controller_nh);
//   this->initCmdSubscriber_(controller_nh);
//   this->initImuSubscriber_(controller_nh);
//   return true;
// }

// //-----------------------------------------------------------------------------
// template<typename OdometryFrameType, typename KinematicType>
// void EnhancedMobileBaseController<OdometryFrameType, KinematicType>::initImuSubscriber_(
//   ros::NodeHandle & controller_nh)
// {
//   imu_sub_ = controller_nh.subscribe<sensor_msgs::Imu>(
//     "imu/data", 1,
//     &EnhancedMobileBaseController::imu_callback_,
//     this);
// }

// //-----------------------------------------------------------------------------
// template<typename OdometryFrameType, typename KinematicType>
// void EnhancedMobileBaseController<OdometryFrameType, KinematicType>::initPID_(
//   ros::NodeHandle & controller_nh)
// {
//   auto angular_speed_pid_nh = ros::NodeHandle(controller_nh, "angular_speed_pid");
//   angular_speed_pid_ = make_pid(angular_speed_pid_nh);
// }

// //-----------------------------------------------------------------------------
// template<typename OdometryFrameType, typename KinematicType>
// void EnhancedMobileBaseController<OdometryFrameType, KinematicType>::update(
//   const ros::Time & time,
//   const ros::Duration & /*period*/)
// {

//   this->update_time_ = time;

// //    ROS_INFO_STREAM_NAMED(this->name_, "update_controller_state_");
//   this->update_controller_state_();
// //    ROS_INFO_STREAM_NAMED(name_, "publish_controller_state_");
//   this->publish_controller_state_();
// //    ROS_INFO_STREAM_NAMED(name_, "read command");
//   this->current_command_ = *(this->command_buffer_.readFromRT());

// //    ROS_INFO_STREAM_NAMED(this->name_, "\n"<<this->current_command_.cmd);

// ROS_INFO_STREAM_NAMED(
//   this->name_,
//   "times " << time << " " << this->current_command_.stamp << " " <<
//     this->previous_command_.stamp);

// //    ROS_INFO_STREAM_NAMED(this->name_, "check timeout");
//   if (this->timeout_()) {
// //        ROS_INFO_STREAM_NAMED(name_, "brake");
//     this->brake_();
//     return;
//   }

// //    ROS_INFO_STREAM_NAMED(name_, "check new command available");

//   if (this->new_command_available_()) {

// ////        ROS_INFO_STREAM_NAMED(name_,"odometry frame measured");
// ////        ROS_INFO_STREAM_NAMED(name_,"\n"<<odometry_frame_);

// ////        ROS_INFO_STREAM_NAMED(name_, " new command ok");
// ////        ROS_INFO_STREAM_NAMED(name_,"\n"<<current_command_.cmd);

// //        clamp_current_command_();
// //        ROS_INFO_STREAM_NAMED(name_, "cnew clamp command");
// //        ROS_INFO_STREAM_NAMED(name_,"\n"<<current_command_.cmd);

// //        send_current_command_();
// ////        ROS_INFO_STREAM_NAMED(name_, "cooucou new command");

//   }
// }

// template class EnhancedMobileBaseController<OdometryFrame4WD, SkidSteeringKinematic>;
// template class EnhancedMobileBaseController<OdometryFrame2WD, SkidSteeringKinematic>;

}  // namespace romea

// PLUGINLIB_EXPORT_CLASS(
//   romea::EnhancedMobileBaseController4WD,
//   controller_interface::ControllerBase);
// PLUGINLIB_EXPORT_CLASS(
//   romea::EnhancedMobileBaseController2WD,
//   controller_interface::ControllerBase);
