// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2TD_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2TD_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// romea core
#include "romea_core_mobile_base/odometry/OdometryFrame2TD.hpp"
#include "romea_core_mobile_base/info/MobileBaseInfo2TD.hpp"

// local
#include "controller_interface_common.hpp"

namespace romea
{

class ControllerInterface2TD
{
public:
  using LoanedCommandInterface = hardware_interface::LoanedCommandInterface;
  using LoanedCommandInterfaces = std::vector<LoanedCommandInterface>;
  using LoanedStateInterface = hardware_interface::LoanedStateInterface;
  using LoanedStateInterfaces = std::vector<LoanedStateInterface>;

  enum JointIds
  {
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID
  };

public:
  explicit ControllerInterface2TD(const MobileBaseInfo2TD & mobile_base_info);

  void write(
    const OdometryFrame2TD & command,
    LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(
    const LoanedStateInterfaces & loaned_state_interfaces,
    OdometryFrame2TD & measurement)const;

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
  double virtual_tracks_radius_;
};


}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE2TD_HPP_
