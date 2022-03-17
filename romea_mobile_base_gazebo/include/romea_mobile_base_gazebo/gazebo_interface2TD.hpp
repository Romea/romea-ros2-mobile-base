#ifndef _romea_GazeboInterface2TD_hpp_
#define _romea_GazeboInterface2TD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface2TD.hpp>

namespace romea
{

struct GazeboInterface2TD{

  using HardwareInterface = HardwareInterface2TD;

  GazeboInterface2TD(gazebo::physics::ModelPtr parent_model,
                     const hardware_interface::HardwareInfo & hardware_info,
                     const std::string & command_interface_type);

  SpinningJointGazeboInterface left_sprocket_wheel_spinning_joint;
  SpinningJointGazeboInterface right_sprocket_wheel_spinning_joint;
  SpinningJointGazeboInterface left_idler_wheel_spinning_joint;
  SpinningJointGazeboInterface right_idler_wheel_spinning_joint;

};

void write(const HardwareInterface2TD & hardware_interface,
           GazeboInterface2TD & gazebo_interface);

void read(const GazeboInterface2TD & gazebo_interface,
          HardwareInterface2TD & hardware_interface);


}

#endif
