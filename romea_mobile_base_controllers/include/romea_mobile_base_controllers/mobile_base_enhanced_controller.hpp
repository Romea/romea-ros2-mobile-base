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

//  #include "odometry_controller.hpp"
//  #include <romea_common_utils/pid_factory.hpp>

//  #include <sensor_msgs/Imu.h>

namespace romea
{

// template<typename OdometryFrameType, typename KinematicType>
// class EnhancedMobileBaseController :
//   public MobileBaseController<OdometryFrameType, KinematicType>
// {

// public:
//   EnhancedMobileBaseController();

//   virtual ~EnhancedMobileBaseController() = default;

//   /**
//    * \brief Initialize controller
//    * \param hw            Velocity joint interface for the wheels
//    * \param root_nh       Node handle at root namespace
//    * \param controller_nh Node handle inside the controller namespace
//    */
//   virtual bool init(
//     hardware_interface::RobotHW * hw,
//     ros::NodeHandle & root_nh,
//     ros::NodeHandle & controller_nh)override;

//   /**
//    * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
//    * \param time   Current time
//    * \param period Time since the last called to update
//    */
//   virtual void update(const ros::Time & time, const ros::Duration & period) override;

// protected:
//   void initImuSubscriber_(ros::NodeHandle & controller_nh);

//   void imu_callback_(const sensor_msgs::Imu::ConstPtr & msg);

//   void initPID_(ros::NodeHandle & controller_nh);

// protected:
//   ros::Subscriber imu_sub_;
//   std::unique_ptr<PID> angular_speed_pid_;
//   std::atomic<double> angular_speed_measure_;

// };

// using EnhancedMobileBaseController4WD = EnhancedMobileBaseController<OdometryFrame4WD,
//     SkidSteeringKinematic>;
// using EnhancedMobileBaseController2WD = EnhancedMobileBaseController<OdometryFrame2WD,
//     SkidSteeringKinematic>;

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__MOBILE_BASE_ENHANCED_CONTROLLER_HPP_
