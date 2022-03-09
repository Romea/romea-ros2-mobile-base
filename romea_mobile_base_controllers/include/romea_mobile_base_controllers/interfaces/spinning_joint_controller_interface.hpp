#ifndef _romea_SpinningJointInterface_hpp_
#define _romea_SpinningJointInterface_hpp_

#include "joint_controller_interface.hpp"

namespace romea
{

class SpinningJointControllerInterface : public JointControllerInterface
{

public :

  SpinningJointControllerInterface(LoanedCommandInterfaces & loaned_command_interfaces,
                                   LoanedStateInterfaces & loaned_state_interfaces,
                                   const std::string & joint_name,
                                   const double & wheel_radius);

  virtual ~SpinningJointControllerInterface()=default;

  virtual void setCommand(const double & command) override;

  virtual double getMeasurement()const override;

private:


  double wheel_radius_;
};

}
#endif
