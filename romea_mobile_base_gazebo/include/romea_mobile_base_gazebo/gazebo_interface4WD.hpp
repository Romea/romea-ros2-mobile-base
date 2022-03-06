#ifndef _romea_GazeboInterface4WD_hpp_
#define _romea_GazeboInterface4WD_hpp_


#include "spinning_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface4WD.hpp>

namespace romea
{

class GazeboInterface4WD{


public:

using HardwareInterface = HardwareInterface4WD;

public:

  GazeboInterface4WD(gazebo::physics::ModelPtr parent_model,
                     const hardware_interface::HardwareInfo & hardware_info,
                     const std::string & command_interface_type);
  SpinningJointGazeboInterface front_left_wheel_spinning_joint;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint;

};

void write(const HardwareInterface4WD & hardware_interface,
           GazeboInterface4WD & gazebo_interface);

void read(const GazeboInterface4WD & gazebo_interface,
          HardwareInterface4WD & hardware_interface);


}

#endif
