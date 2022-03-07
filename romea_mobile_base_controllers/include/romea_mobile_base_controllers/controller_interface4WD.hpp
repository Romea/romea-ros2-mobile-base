#ifndef _romea_ControllerInterface4WD_hpp_
#define _romea_ControllerInterface4WD_hpp_

#include "spinning_joint_controller_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame4WD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo4WD.hpp>

namespace romea
{

class ControllerInterface4WD
{

public:

  using LoanedCommandInterfaces = JointControllerInterface::LoanedCommandInterfaces;
  using LoanedStateInterfaces = JointControllerInterface::LoanedStateInterfaces;

public:

  ControllerInterface4WD(const MobileBaseInfo4WD & mobile_base_info,
                         const std::map<std::string,std::string> & joint_mappings,
                         LoanedCommandInterfaces & loaned_command_interfaces,
                         LoanedStateInterfaces & loaned_state_interfaces);

  void setCommand(const OdometryFrame4WD & cmd);

  OdometryFrame4WD getOdometryFrame() const;

  std::vector<std::string> getCommandInterfaceNames()const;

  std::vector<std::string> getStateInterfaceNames()const;

private :

  SpinningJointControllerInterface front_left_spinning_joint_;
  SpinningJointControllerInterface front_right_spinning_joint_;
  SpinningJointControllerInterface rear_left_spinning_joint_;
  SpinningJointControllerInterface rear_right_spinning_joint_;

};


}

#endif