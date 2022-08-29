#ifndef _romea_GazeboInterface2AS4WD_hpp_
#define _romea_GazeboInterface2AS4WD_hpp_

//romea
#include "spinning_joint_gazebo_interface.hpp"
#include "steering_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface2AS4WD.hpp>

namespace romea
{

class GazeboInterface2AS4WD{


public:

using HardwareInterface = HardwareInterface2AS4WD;

public:

  GazeboInterface2AS4WD(gazebo::physics::ModelPtr parent_model,
                        const hardware_interface::HardwareInfo & hardware_info,
                        const std::string & command_interface_type);

  SteeringJointGazeboInterface front_axle_steering_joint;
  SteeringJointGazeboInterface rear_axle_steering_joint;
  SteeringJointGazeboInterface front_left_wheel_steering_joint;
  SteeringJointGazeboInterface front_right_wheel_steering_joint;
  SteeringJointGazeboInterface rear_left_wheel_steering_joint;
  SteeringJointGazeboInterface rear_right_wheel_steering_joint;
  SpinningJointGazeboInterface front_left_wheel_spinning_joint;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint;

  const double wheelbase;
  const double front_track;
  const double rear_track;
};

void write(const HardwareInterface2AS4WD & hardware_interface,
           GazeboInterface2AS4WD & gazebo_interface);

void read(const GazeboInterface2AS4WD & gazebo_interface,
          HardwareInterface2AS4WD & hardware_interface);


}

#endif
