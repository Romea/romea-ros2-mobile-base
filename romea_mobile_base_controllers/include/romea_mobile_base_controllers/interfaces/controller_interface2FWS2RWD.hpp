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


#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2FWS2RWD_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2FWS2RWD_HPP_


// std
#include <memory>
#include <string>
#include <vector>

// romea core
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS2RWD.hpp"
#include "romea_core_mobile_base/info/MobileBaseInfo2FWS2RWD.hpp"

// local
#include "controller_interface_common.hpp"

namespace romea
{

class ControllerInterface2FWS2RWD
{
public:
  using LoanedCommandInterface = hardware_interface::LoanedCommandInterface;
  using LoanedCommandInterfaces = std::vector<LoanedCommandInterface>;
  using LoanedStateInterface = hardware_interface::LoanedStateInterface;
  using LoanedStateInterfaces = std::vector<LoanedStateInterface>;

  enum JointIds
  {
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID,
  };

public:
  explicit ControllerInterface2FWS2RWD(const MobileBaseInfo2FWS2RWD & mobile_base_info);

  void write(
    const OdometryFrame2FWS2RWD & command,
    LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(
    const LoanedStateInterfaces & loaned_state_interfaces,
    OdometryFrame2FWS2RWD & measurement)const;

public:
  static void declare_joints_names(
    std::shared_ptr<HardwareInterfaceNode> node,
    const std::string & parameters_ns);

  static std::vector<std::string> get_joints_names(
    std::shared_ptr<HardwareInterfaceNode> node,
    const std::string & parameters_ns);

  static std::vector<std::string> hardware_interface_names(
    const std::vector<std::string> & joints_names);

private:
  double front_wheel_radius_;
  double rear_wheel_radius_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2FWS2RWD_HPP_
