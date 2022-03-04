#ifndef _romea_GazeboInterface1FAS2FWD_hpp_
#define _romea_GazeboInterface1FAS2FWD_hpp_


#include "spinning_joint_gazebo_interface.hpp"
#include "steering_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface1FAS2FWD.hpp>

namespace romea
{

class GazeboInterface1FAS2FWD{

public:

  GazeboInterface1FAS2FWD(gazebo::physics::ModelPtr parent_model,
                          const hardware_interface::HardwareInfo & hardware_info,
                          const std::string & command_interface_type);
  SteeringJointGazeboInterface front_axle_steering_joint;
  SteeringJointGazeboInterface front_left_wheel_steering_joint;
  SteeringJointGazeboInterface front_right_wheel_steering_joint;
  SpinningJointGazeboInterface front_left_wheel_spinning_joint;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint;

  const double wheelbase;
  const double front_track;
  const double rear_track;
};

void write(const HardwareInterface1FAS2FWD & hardware_interface,
           GazeboInterface1FAS2FWD & gazebo_interface);

void read(const GazeboInterface1FAS2FWD & gazebo_interface,
          HardwareInterface1FAS2FWD & hardware_interface);


}

#endif
