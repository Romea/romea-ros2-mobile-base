// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2FWS2FWD_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2FWS2FWD_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// romea core
#include "romea_core_mobile_base/odometry/OdometryFrame2FWS2FWD.hpp"
#include "romea_core_mobile_base/info/MobileBaseInfo2FWS2FWD.hpp"

// local
#include "controller_interface_common.hpp"

namespace romea
{

class ControllerInterface2FWS2FWD
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
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID,
  };

public:
  explicit ControllerInterface2FWS2FWD(const MobileBaseInfo2FWS2FWD & mobile_base_info);

  void write(
    const OdometryFrame2FWS2FWD & command,
    LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(
    const LoanedStateInterfaces & loaned_state_interfaces,
    OdometryFrame2FWS2FWD & measurement)const;

public:
  static void declare_joints_names(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & parameters_ns);

  static std::vector<std::string> get_joints_names(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & parameters_ns);

  static std::vector<std::string> hardware_interface_names(
    const std::vector<std::string> & joints_names);

private:
  double front_wheels_radius_;
};


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2FWS2FWD_HPP_
