#ifndef _romea_ControllerInterface1FWS2RWD_hpp_
#define _romea_ControllerInterface1FWS2RWD_hpp_

//romea
#include "steering_joint_controller_interface.hpp"
#include "spinning_joint_controller_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame1FWS2RWD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo1FWS2RWD.hpp>

namespace romea
{

class ControllerInterface1FWS2RWD
{

public:

  using LoanedCommandInterfaces = JointControllerInterface::LoanedCommandInterfaces;
  using LoanedStateInterfaces = JointControllerInterface::LoanedStateInterfaces;

public:

  ControllerInterface1FWS2RWD(const MobileBaseInfo1FWS2RWD & mobile_base_info,
                              const std::map<std::string,std::string> & joint_mappings,
                              LoanedCommandInterfaces & loaned_command_interfaces,
                              LoanedStateInterfaces & loaned_state_interfaces);

  void setCommand(const OdometryFrame1FWS2RWD & cmd);

  OdometryFrame1FWS2RWD getOdometryFrame() const;

  std::vector<std::string> getCommandInterfaceNames()const;

  std::vector<std::string> getStateInterfaceNames()const;

private :

  SteeringJointControllerInterface front_steering_joint_;
  SpinningJointControllerInterface rear_left_spinning_joint_;
  SpinningJointControllerInterface rear_right_spinning_joint_;

};


}

#endif
