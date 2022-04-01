#ifndef _romea_ControllerInterface2AS4WD_hpp_
#define _romea_ControllerInterface2AS4WD_hpp_


//romea
#include "steering_joint_controller_interface.hpp"
#include "spinning_joint_controller_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame2AS4WD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2AS4WD.hpp>
namespace romea
{

class ControllerInterface2AS4WD
{
public:

  using LoanedCommandInterfaces = JointControllerInterface::LoanedCommandInterfaces;
  using LoanedStateInterfaces = JointControllerInterface::LoanedStateInterfaces;

  enum JointIds {
    FRONT_AXLE_STEERING_JOINT_ID,
    REAR_AXLE_STEERING_JOINT_ID,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID
  };

public:

  ControllerInterface2AS4WD(const MobileBaseInfo2AS4WD & mobile_base_info);

  void write(const OdometryFrame2AS4WD & command,
             LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(const LoanedStateInterfaces & loaned_state_interfaces,
            OdometryFrame2AS4WD & measurement)const;

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

  SteeringJointControllerInterface steering_joints_;
  SpinningJointControllerInterface front_spinning_joints_;
  SpinningJointControllerInterface rear_spinning_joints_;

};


}

#endif
