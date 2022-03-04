#ifndef _romea_GazeboInterface2FWS2RWD_hpp_
#define _romea_GazeboInterface2FWS2RWD_hpp_


#include "spinning_joint_gazebo_interface.hpp"
#include "steering_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface2FWS2RWD.hpp>

namespace romea
{

class GazeboInterface2FWS2RWD{

public:

  GazeboInterface2FWS2RWD(gazebo::physics::ModelPtr parent_model,
                        const hardware_interface::HardwareInfo & hardware_info,
                        const std::string & command_interface_type);
  SteeringJointGazeboInterface front_left_wheel_steering_joint;
  SteeringJointGazeboInterface front_right_wheel_steering_joint;
  SpinningJointGazeboInterface front_left_wheel_spinning_joint;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint;

  const double wheelbase;
  const double front_track;
  const double front_hub_carrier_offset;
};

void write(const HardwareInterface2FWS2RWD & hardware_interface,
           GazeboInterface2FWS2RWD & gazebo_interface);

void read(const GazeboInterface2FWS2RWD & gazebo_interface,
          HardwareInterface2FWS2RWD & hardware_interface);


}

#endif