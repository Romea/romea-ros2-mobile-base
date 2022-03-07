#ifndef _romea_SteeringJointInterface_hpp_
#define _romea_SteeringJointInterface_hpp_

#include "joint_controller_interface.hpp"

namespace romea
{

class SteeringJointControllerInterface : public JointControllerInterface
{

public :

  SteeringJointControllerInterface(LoanedCommandInterfaces & loaned_command_interfaces,
                                   LoanedStateInterfaces & loaned_state_interfaces,
                                   const std::string & joint_name);

  virtual ~SteeringJointControllerInterface()=default;

  virtual double getMeasurement()const override;

  virtual void setCommand(const double & command) override;

};

}

#endif


