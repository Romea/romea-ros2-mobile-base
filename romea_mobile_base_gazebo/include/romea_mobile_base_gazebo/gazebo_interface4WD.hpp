#ifndef _romea_GazeboInterface4WD_hpp_
#define _romea_GazeboInterface4WD_hpp_


#include "spinning_joint_gazebo_interface.hpp"
#include <romea_core_mobile_base/simulation/SimulationControl4WD.hpp>

namespace romea
{

class GazeboInterface4WD{

public:

  GazeboInterface4WD(gazebo::physics::ModelPtr parent_model,
                     const hardware_interface::HardwareInfo & hardware_info,
                     const std::string & command_interface_type);

  SimulationState4WD get_state() const;
  void set_command(const SimulationCommand4WD &command);

private :

  SpinningJointGazeboInterface front_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface front_right_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_left_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_right_wheel_spinning_joint_;

};

}

#endif
