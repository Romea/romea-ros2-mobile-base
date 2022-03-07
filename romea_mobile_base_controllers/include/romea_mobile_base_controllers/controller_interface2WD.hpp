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

public:

  ControllerInterface2WD(const MobileBaseInfo2WD & mobile_base_info,
                         const std::map<std::string,std::string> & joint_mappings,
                         LoanedCommandInterfaces & loaned_command_interfaces,
                         LoanedStateInterfaces & loaned_state_interfaces);

  void setCommand(const OdometryFrame2WD & cmd);

  OdometryFrame2WD getOdometryFrame() const;

  std::vector<std::string> getCommandInterfaceNames()const;

  std::vector<std::string> getStateInterfaceNames()const;

private :

  SpinningJointControllerInterface left_spinning_joint_;
  SpinningJointControllerInterface right_spinning_joint_;

};


}

#endif
