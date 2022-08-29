#ifndef _romea_GazeboInterface2TTD_hpp_
#define _romea_GazeboInterface2TTD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include <romea_core_mobile_base/simulation/SimulationControl2TTD.hpp>

namespace romea
{

class GazeboInterface2TTD{


public :

  GazeboInterface2TTD(gazebo::physics::ModelPtr parent_model,
                      const hardware_interface::HardwareInfo & hardware_info,
                      const std::string & command_interface_type);

  SimulationState2TTD get_state() const;
  void set_command(const SimulationCommand2TTD &command);

private:

  SpinningJointGazeboInterface left_sprocket_wheel_spinning_joint_;
  SpinningJointGazeboInterface right_sprocket_wheel_spinning_joint_;
  SpinningJointGazeboInterface left_idler_wheel_spinning_joint_;
  SpinningJointGazeboInterface right_idler_wheel_spinning_joint_;
  SpinningJointGazeboInterface front_left_roller_wheel_spinning_joint_;
  SpinningJointGazeboInterface front_right_roller_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_left_roller_wheel_spinning_joint_;
  SpinningJointGazeboInterface rear_right_roller_wheel_spinning_joint_;

};

}

#endif
