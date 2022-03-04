#include "romea_mobile_base_gazebo/gazebo_interface4WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface4WD::GazeboInterface4WD(gazebo::physics::ModelPtr parent_model,
                                       const hardware_interface::HardwareInfo & hardware_info,
                                       const std::string & command_interface_type):
  front_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface4WD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface4WD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface4WD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface4WD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{

}


//-----------------------------------------------------------------------------
void write(const HardwareInterface4WD & hardware_interface,
           GazeboInterface4WD & gazebo_interface)
{
  write(hardware_interface.front_left_wheel_spinning_joint,
        gazebo_interface.front_left_wheel_spinning_joint);
  write(hardware_interface.front_right_wheel_spinning_joint,
        gazebo_interface.front_right_wheel_spinning_joint);
  write(hardware_interface.rear_left_wheel_spinning_joint,
        gazebo_interface.rear_left_wheel_spinning_joint);
  write(hardware_interface.rear_right_wheel_spinning_joint,
        gazebo_interface.rear_right_wheel_spinning_joint);

}

//-----------------------------------------------------------------------------
void read(const GazeboInterface4WD & gazebo_interface,
          HardwareInterface4WD & hardware_interface)
{
  read(gazebo_interface.front_left_wheel_spinning_joint,
       hardware_interface.front_left_wheel_spinning_joint);
  read(gazebo_interface.front_right_wheel_spinning_joint,
       hardware_interface.front_right_wheel_spinning_joint);
  read(gazebo_interface.rear_left_wheel_spinning_joint,
       hardware_interface.rear_left_wheel_spinning_joint);
  read(gazebo_interface.rear_right_wheel_spinning_joint,
       hardware_interface.rear_right_wheel_spinning_joint);

}


}
