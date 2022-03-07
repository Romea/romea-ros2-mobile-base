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

public:

  ControllerInterface1FAS2RWD(const MobileBaseInfo1FAS2RWD & mobile_base_info,
                              const std::map<std::string,std::string> & joint_mappings,
                              LoanedCommandInterfaces & loaned_command_interfaces,
                              LoanedStateInterfaces & loaned_state_interfaces);

  void setCommand(const OdometryFrame1FAS2RWD & cmd);

  OdometryFrame1FAS2RWD getOdometryFrame() const;

  std::vector<std::string> getCommandInterfaceNames()const;

  std::vector<std::string> getStateInterfaceNames()const;

private :

  SteeringJointControllerInterface front_steering_joint_;
  SpinningJointControllerInterface rear_left_spinning_joint_;
  SpinningJointControllerInterface rear_right_spinning_joint_;

};


}

#endif
