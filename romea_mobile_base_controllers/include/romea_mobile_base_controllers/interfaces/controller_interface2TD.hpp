#ifndef _romea_ControllerInterface2TD_hpp_
#define _romea_ControllerInterface2TD_hpp_

#include "spinning_joint_controller_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame2WD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2TD.hpp>

namespace romea
{

class ControllerInterface2TD
{

public:

  using LoanedCommandInterfaces = JointControllerInterface::LoanedCommandInterfaces;
  using LoanedStateInterfaces = JointControllerInterface::LoanedStateInterfaces;

  enum JointIds {
     LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
     RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID
  };

public:

  ControllerInterface2TD(const MobileBaseInfo2TD & mobile_base_info,
                         const std::map<int,std::string> & joint_mappings,
                         LoanedCommandInterfaces & loaned_command_interfaces,
                         LoanedStateInterfaces & loaned_state_interfaces);

  void setCommand(const OdometryFrame2WD & cmd);

  OdometryFrame2WD getOdometryFrame() const;

  std::vector<std::string> getCommandInterfaceNames()const;

  std::vector<std::string> getStateInterfaceNames()const;

  static void declare_joints_mapping(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & parameters_ns);

  static std::map<int,std::string> get_joints_mapping(std::shared_ptr<rclcpp::Node> node,
                                                      const std::string & parameters_ns);

private :

  SpinningJointControllerInterface left_sprocket_spinning_joint_;
  SpinningJointControllerInterface right_sprocket_spinning_joint_;

};


}

#endif
