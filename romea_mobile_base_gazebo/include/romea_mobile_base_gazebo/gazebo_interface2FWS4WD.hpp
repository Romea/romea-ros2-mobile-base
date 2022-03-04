#ifndef _romea_GazeboInterface2FWS4WD_hpp_
#define _romea_GazeboInterface2FWS4WD_hpp_


#include "spinning_joint_gazebo_interface.hpp"
#include "steering_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface2FWS4WD.hpp>

namespace romea
{

class GazeboInterface2FWS4WD{

public:

  GazeboInterface2FWS4WD(gazebo::physics::ModelPtr parent_model,
                        const hardware_interface::HardwareInfo & hardware_info,
                        const std::string & command_interface_type);
  SteeringJointGazeboInterface front_left_wheel_steering_joint;
  SteeringJointGazeboInterface front_right_wheel_steering_joint;
  SpinningJointGazeboInterface front_left_wheel_spinning_joint;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint;

};

void write(const HardwareInterface2FWS4WD & hardware_interface,
           GazeboInterface2FWS4WD & gazebo_interface);

void read(const GazeboInterface2FWS4WD & gazebo_interface,
          HardwareInterface2FWS4WD & hardware_interface);


}

#endif
