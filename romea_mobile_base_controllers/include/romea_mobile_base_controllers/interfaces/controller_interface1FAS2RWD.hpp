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

  ControllerInterface1FAS2RWD(const MobileBaseInfo1FAS2RWD & mobile_base_info);

  void write(const OdometryFrame1FAS2RWD & command,
             LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(const LoanedStateInterfaces & loaned_state_interfaces,
            OdometryFrame1FAS2RWD & measurement)const;

public:

  static void declare_joints_names(
      std::shared_ptr<rclcpp::Node> node,
      const std::string & parameters_ns);

  static std::vector<std::string> get_joints_names(
      std::shared_ptr<rclcpp::Node> node,
      const std::string & parameters_ns);

  static std::vector<std::string> hardware_interface_names(
      const std::vector<std::string> & joints_names);

private :

  SteeringJointControllerInterface front_steering_joint_;
  SpinningJointControllerInterface rear_spinning_joints_;

};


}

#endif
