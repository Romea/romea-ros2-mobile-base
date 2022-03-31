#ifndef _romea_SpinningJointInterface_hpp_
#define _romea_SpinningJointInterface_hpp_

#include "joint_controller_interface.hpp"

namespace romea
{

class SpinningJointControllerInterface : public JointControllerInterface
{

public :

  SpinningJointControllerInterface(const std::string & joint_name,
                                   const double & wheel_radius);

  virtual ~SpinningJointControllerInterface()=default;

  virtual void set_command(const double & command) override;

  virtual double get_measurement()const override;

private:


  double wheel_radius_;
};

}
#endif
