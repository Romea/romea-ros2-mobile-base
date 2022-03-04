#ifndef _romea_SteeringJointGazeboInterface_hpp_
#define _romea_SteeringJointGazeboInterface_hpp_

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"

#include <romea_mobile_base_hardware/steering_joint_hardware_interface.hpp>

namespace romea
{

class SteeringJointGazeboInterface
{

public:

  SteeringJointGazeboInterface(gazebo::physics::ModelPtr parent_model,
                               const hardware_interface::ComponentInfo & joint_info);

  void setCommand(const double & command);

  double getFeedback()const;

private:

  gazebo::physics::JointPtr sim_joint_;

};

void write(const SteeringJointHardwareInterface & hardware_joint,
           SteeringJointGazeboInterface & gazebo_joint);

void read(const SteeringJointGazeboInterface & gazebo_joint,
          SteeringJointHardwareInterface & hardware_joint);

void read(const SteeringJointGazeboInterface & gazebo_joint,
          SteeringJointHardwareInterface::Feedback & hardware_joint_feedback);

}
#endif
