#ifndef _romea_SteeringJointInterface_hpp_
#define _romea_SteeringJointInterface_hpp_

#include "joint_controller_interface.hpp"

namespace romea
{

class SteeringJointControllerInterface : public JointControllerInterface
{

public :

  SteeringJointControllerInterface(const std::string & joint_name);

  virtual ~SteeringJointControllerInterface()=default;

  virtual double get_measurement()const override;

  virtual void set_command(const double & command) override;

};

}

#endif


