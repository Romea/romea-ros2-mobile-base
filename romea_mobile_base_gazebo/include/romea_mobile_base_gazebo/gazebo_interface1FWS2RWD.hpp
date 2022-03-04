#ifndef _romea_GazeboInterface1FWS2RWD_hpp_
#define _romea_GazeboInterface1FWS2RWD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include "steering_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface1FWS2RWD.hpp>

namespace romea
{

class GazeboInterface1FWS2RWD{

public:

  GazeboInterface1FWS2RWD(gazebo::physics::ModelPtr parent_model,
                          const hardware_interface::HardwareInfo & hardware_info,
                          const std::string & command_interface_type);

  SteeringJointGazeboInterface front_wheel_steering_joint;
  SpinningJointGazeboInterface front_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint;

};

void write(const HardwareInterface1FWS2RWD & hardware_interface,
           GazeboInterface1FWS2RWD & gazebo_interface);

void read(const GazeboInterface1FWS2RWD & gazebo_interface,
          HardwareInterface1FWS2RWD & hardware_interface);


}

#endif
