#ifndef _romea_ControllerInterface2WD_hpp_
#define _romea_ControllerInterface2WD_hpp_

#include "spinning_joint_controller_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame2WD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2WD.hpp>
namespace romea
{

class ControllerInterface2WD
{

public:

  using LoanedCommandInterfaces = JointControllerInterface::LoanedCommandInterfaces;
  using LoanedStateInterfaces = JointControllerInterface::LoanedStateInterfaces;

  enum JointIds {
     LEFT_WHEEL_SPINNING_JOINT_ID,
     RIGHT_WHEEL_SPINNING_JOINT_ID
  };

public:

  ControllerInterface2WD(const MobileBaseInfo2WD & mobile_base_info,
                         const std::vector<std::string> & joints_names);

  void set_command(const OdometryFrame2WD & cmd);

  OdometryFrame2WD get_odometry_frame() const;

  std::vector<std::string> get_command_interface_names()const;

  std::vector<std::string> get_state_interface_names()const;

  void register_loaned_command_interfaces(LoanedCommandInterfaces & loaned_command_interfaces);

  void register_loaned_state_interfaces(LoanedStateInterfaces & loaned_state_interfaces);

public :

  static void declare_joints_names(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & parameters_ns);

  static std::vector<std::string> get_joints_names(std::shared_ptr<rclcpp::Node> node,
                                                      const std::string & parameters_ns);


private :

  SpinningJointControllerInterface left_spinning_joint_;
  SpinningJointControllerInterface right_spinning_joint_;

};


}

#endif
