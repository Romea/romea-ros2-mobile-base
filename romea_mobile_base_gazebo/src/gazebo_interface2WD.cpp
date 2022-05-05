#include "romea_mobile_base_gazebo/gazebo_interface2WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2WD::GazeboInterface2WD(gazebo::physics::ModelPtr parent_model,
                                       const hardware_interface::HardwareInfo & hardware_info,
                                       const std::string & command_interface_type):
  left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2WD::LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2WD::RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{

}

//-----------------------------------------------------------------------------
void write(const HardwareInterface2WD & hardware_interface,
           GazeboInterface2WD & gazebo_interface)
{
  write(hardware_interface.left_wheel_spinning_joint,
        gazebo_interface.left_wheel_spinning_joint);
  write(hardware_interface.right_wheel_spinning_joint,
        gazebo_interface.right_wheel_spinning_joint);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface2WD & gazebo_interface,
          HardwareInterface2WD & hardware_interface)
{
  read(gazebo_interface.left_wheel_spinning_joint,
       hardware_interface.left_wheel_spinning_joint);
  read(gazebo_interface.right_wheel_spinning_joint,
       hardware_interface.right_wheel_spinning_joint);
}

}


