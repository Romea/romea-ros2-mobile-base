#ifndef _romea_GazeboInterface2THD_hpp_
#define _romea_GazeboInterface2THD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface2THD.hpp>

namespace romea
{

struct GazeboInterface2THD{

  using HardwareInterface = HardwareInterface2THD;

  GazeboInterface2THD(gazebo::physics::ModelPtr parent_model,
                      const hardware_interface::HardwareInfo & hardware_info,
                      const std::string & command_interface_type);

  SpinningJointGazeboInterface left_sprocket_wheel_spinning_joint;
  SpinningJointGazeboInterface right_sprocket_wheel_spinning_joint;
  SpinningJointGazeboInterface front_left_idler_wheel_spinning_joint;
  SpinningJointGazeboInterface front_right_idler_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_left_idler_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_right_idler_wheel_spinning_joint;

  const double idler_wheel_radius;
  const double sprocket_wheel_radius;
};

void write(const HardwareInterface2THD & hardware_interface,
           GazeboInterface2THD & gazebo_interface);

void read(const GazeboInterface2THD & gazebo_interface,
          HardwareInterface2THD & hardware_interface);


}

#endif
