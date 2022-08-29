#ifndef _romea_GazeboInterface2TD_hpp_
#define _romea_GazeboInterface2TD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include <romea_core_mobile_base/simulation/SimulationControl2TD.hpp>

namespace romea
{

class GazeboInterface2TD{

public:

  GazeboInterface2TD(gazebo::physics::ModelPtr parent_model,
                     const hardware_interface::HardwareInfo & hardware_info,
                     const std::string & command_interface_type);

  SimulationState2TD get_state() const;
  void set_command(const SimulationCommand2TD &command);

private :

  SpinningJointGazeboInterface left_sprocket_wheel_spinning_joint_;
  SpinningJointGazeboInterface right_sprocket_wheel_spinning_joint_;
  SpinningJointGazeboInterface left_idler_wheel_spinning_joint_;
  SpinningJointGazeboInterface right_idler_wheel_spinning_joint_;

};

}

#endif
