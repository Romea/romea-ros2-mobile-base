#ifndef _romea_GazeboInterface1FWS2RWD_hpp_
#define _romea_GazeboInterface1FWS2RWD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include "steering_joint_gazebo_interface.hpp"
#include <romea_core_mobile_base/simulation/SimulationControl1FWS2RWD.hpp>

namespace romea
{

struct GazeboInterface1FWS2RWD{

public :

  GazeboInterface1FWS2RWD(gazebo::physics::ModelPtr parent_model,
                          const hardware_interface::HardwareInfo & hardware_info,
                          const std::string & command_interface_type);

  SimulationState1FWS2RWD get_state() const;
  void set_command(const SimulationCommand1FWS2RWD & command);

private :

  SteeringJointGazeboInterface front_wheel_steering_joint_;
  SpinningJointGazeboInterface front_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint_;

};

}

#endif
