// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE1FAS2FWD_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE1FAS2FWD_HPP_

// romea_core
#include <romea_core_mobile_base/odometry/OdometryFrame1FAS2FWD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo1FAS2FWD.hpp>

// std
#include <memory>
#include <string>
#include <vector>

// local
#include "controller_interface_common.hpp"

namespace romea
{

class ControllerInterface1FAS2FWD
{
public:
  using LoanedCommandInterface = hardware_interface::LoanedCommandInterface;
  using LoanedCommandInterfaces = std::vector<LoanedCommandInterface>;
  using LoanedStateInterface = hardware_interface::LoanedStateInterface;
  using LoanedStateInterfaces = std::vector<LoanedStateInterface>;


  enum JointIds
  {
    FRONT_AXLE_STEERING_JOINT_ID,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID
  };

public:
  explicit ControllerInterface1FAS2FWD(const MobileBaseInfo1FAS2FWD & mobile_base_info);

  void write(
    const OdometryFrame1FAS2FWD & command,
    LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(
    const LoanedStateInterfaces & loaned_state_interfaces,
    OdometryFrame1FAS2FWD & measurement)const;

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
  double front_wheel_radius_;
};


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE1FAS2FWD_HPP_
