#ifndef _romea_GazeboInterface2TTD_hpp_
#define _romea_GazeboInterface2TTD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface2TTD.hpp>

namespace romea
{

struct GazeboInterface2TTD{

  using HardwareInterface = HardwareInterface2TTD;

  GazeboInterface2TTD(gazebo::physics::ModelPtr parent_model,
                      const hardware_interface::HardwareInfo & hardware_info,
                      const std::string & command_interface_type);

  SpinningJointGazeboInterface left_sprocket_wheel_spinning_joint;
  SpinningJointGazeboInterface right_sprocket_wheel_spinning_joint;
  SpinningJointGazeboInterface left_idler_wheel_spinning_joint;
  SpinningJointGazeboInterface right_idler_wheel_spinning_joint;
  SpinningJointGazeboInterface front_left_roller_wheel_spinning_joint;
  SpinningJointGazeboInterface front_right_roller_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_left_roller_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_right_roller_wheel_spinning_joint;

  const double roller_wheel_radius;
  const double sprocket_wheel_radius;
};

void write(const HardwareInterface2TTD & hardware_interface,
           GazeboInterface2TTD & gazebo_interface);

void read(const GazeboInterface2TTD & gazebo_interface,
          HardwareInterface2TTD & hardware_interface);


}

#endif
