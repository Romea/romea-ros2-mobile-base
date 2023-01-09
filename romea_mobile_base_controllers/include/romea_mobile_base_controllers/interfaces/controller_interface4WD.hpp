// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE4WD_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE4WD_HPP_

// romea core
#include <romea_core_mobile_base/odometry/OdometryFrame4WD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo4WD.hpp>

// std
#include <memory>
#include <string>
#include <vector>

// local
#include "controller_interface_common.hpp"

namespace romea
{

class ControllerInterface4WD
{
public:
  using LoanedCommandInterface = hardware_interface::LoanedCommandInterface;
  using LoanedCommandInterfaces = std::vector<LoanedCommandInterface>;
  using LoanedStateInterface = hardware_interface::LoanedStateInterface;
  using LoanedStateInterfaces = std::vector<LoanedStateInterface>;

  enum JointIds
  {
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID
  };

public:
  explicit ControllerInterface4WD(const MobileBaseInfo4WD & mobile_base_info);

  void write(
    const OdometryFrame4WD & command,
    LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(
    const LoanedStateInterfaces & loaned_state_interfaces,
    OdometryFrame4WD & measurement)const;

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
  double rear_wheels_radius_;
};


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE4WD_HPP_
