#ifndef _romea_GazeboInterface1FAS2RWD_hpp_
#define _romea_GazeboInterface1FAS2RWD_hpp_


#include "spinning_joint_gazebo_interface.hpp"
#include "steering_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface1FAS2RWD.hpp>

namespace romea
{

struct GazeboInterface1FAS2RWD{

  using HardwareInterface = HardwareInterface1FAS2RWD;

  GazeboInterface1FAS2RWD(gazebo::physics::ModelPtr parent_model,
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
  const double front_hub_carrier_offset;
};

void write(const HardwareInterface1FAS2RWD & hardware_interface,
           GazeboInterface1FAS2RWD & gazebo_interface);

void read(const GazeboInterface1FAS2RWD & gazebo_interface,
          HardwareInterface1FAS2RWD & hardware_interface);


}

#endif
