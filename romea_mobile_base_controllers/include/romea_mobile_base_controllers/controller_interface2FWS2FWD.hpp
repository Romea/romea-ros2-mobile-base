#ifndef _romea_ControllerInterface2FWS2FWD_hpp_
#define _romea_ControllerInterface2FWS2FWD_hpp_

//romea
#include "steering_joint_controller_interface.hpp"
#include "spinning_joint_controller_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame2FWS2FWD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2FWS2FWD.hpp>

namespace romea
{

class ControllerInterface2FWS2FWD
{

public:

  using LoanedCommandInterfaces = JointControllerInterface::LoanedCommandInterfaces;
  using LoanedStateInterfaces = JointControllerInterface::LoanedStateInterfaces;

public:

  ControllerInterface2FWS2FWD(const MobileBaseInfo2FWS2FWD & mobile_base_info,
                              const std::map<std::string,std::string> & joint_mappings,
                              LoanedCommandInterfaces & loaned_command_interfaces,
                              LoanedStateInterfaces & loaned_state_interfaces);

  void setCommand(const OdometryFrame2FWS2FWD & cmd);

  OdometryFrame2FWS2FWD getOdometryFrame() const;

  std::vector<std::string> getCommandInterfaceNames()const;

  std::vector<std::string> getStateInterfaceNames()const;

private :

  SteeringJointControllerInterface front_left_steering_joint_;
  SteeringJointControllerInterface front_right_steering_joint_;
  SpinningJointControllerInterface front_left_spinning_joint_;
  SpinningJointControllerInterface front_right_spinning_joint_;

};


}

#endif
