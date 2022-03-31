#ifndef _romea_ControllerInterface1FAS2RWD_hpp_
#define _romea_ControllerInterface1FAS2RWD_hpp_

#include "steering_joint_controller_interface.hpp"
#include "spinning_joint_controller_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame1FAS2RWD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo1FAS2RWD.hpp>

namespace romea
{

class ControllerInterface1FAS2RWD
{
public:

  using LoanedCommandInterfaces = JointControllerInterface::LoanedCommandInterfaces;
  using LoanedStateInterfaces = JointControllerInterface::LoanedStateInterfaces;

  enum JointIds
  {
    FRONT_AXLE_STEERING_JOINT_ID,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID
  };

public:

  ControllerInterface1FAS2RWD(const MobileBaseInfo1FAS2RWD & mobile_base_info,
                              const std::vector<std::string> & joints_names);

  void set_command(const OdometryFrame1FAS2RWD & cmd);

  OdometryFrame1FAS2RWD get_odometry_frame() const;

  std::vector<std::string> get_command_interface_names()const;

  std::vector<std::string> get_state_interface_names()const;

  void register_loaned_command_interfaces(LoanedCommandInterfaces & loaned_command_interfaces);

  void register_loaned_state_interfaces(LoanedStateInterfaces & loaned_state_interfaces);

public:

  static void declare_joints_names(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & parameters_ns);

  static std::vector<std::string> get_joints_names(std::shared_ptr<rclcpp::Node> node,
                                                      const std::string & parameters_ns);

private :

  SteeringJointControllerInterface front_steering_joint_;
  SpinningJointControllerInterface rear_left_spinning_joint_;
  SpinningJointControllerInterface rear_right_spinning_joint_;

};


}

#endif
