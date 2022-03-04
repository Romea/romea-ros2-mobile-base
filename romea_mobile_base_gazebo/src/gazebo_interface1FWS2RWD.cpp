#include "romea_mobile_base_gazebo/gazebo_interface1FWS2RWD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface1FWS2RWD::GazeboInterface1FWS2RWD(gazebo::physics::ModelPtr parent_model,
                                                 const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & command_interface_type):
  front_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface1FWS2RWD::FRONT_WHEEL_STEERING_JOINT_ID]),
  front_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FWS2RWD::FRONT_WHEEL_STEERING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FWS2RWD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FWS2RWD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{

}

//-----------------------------------------------------------------------------
void write(const HardwareInterface1FWS2RWD & hardware_interface,
           const double & front_wheel_speed_command,
           GazeboInterface1FWS2RWD & gazebo_interface)
{

  write(hardware_interface.front_wheel_steering_joint,
        gazebo_interface.front_wheel_steering_joint);

  write(hardware_interface.rear_left_wheel_spinning_joint,
        gazebo_interface.rear_left_wheel_spinning_joint);
  write(hardware_interface.rear_right_wheel_spinning_joint,
        gazebo_interface.rear_right_wheel_spinning_joint);

  gazebo_interface.front_wheel_spinning_joint.
      setCommand(front_wheel_speed_command);

}
//-----------------------------------------------------------------------------
void write(const HardwareInterface1FWS2RWD & hardware_interface,
           GazeboInterface1FWS2RWD & gazebo_interface)
{
  double linear_speed_command =
     (hardware_interface.rear_left_wheel_spinning_joint.command.get()+
      hardware_interface.rear_right_wheel_spinning_joint.command.get())/2.0;

  double front_wheel_speed_command = linear_speed_command*
      std::cos(hardware_interface.front_wheel_steering_joint.command.get());

  write(hardware_interface,front_wheel_speed_command,gazebo_interface);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface1FWS2RWD & gazebo_interface,
          HardwareInterface1FWS2RWD & hardware_interface)
{
  read(gazebo_interface.front_wheel_steering_joint,
       hardware_interface.front_wheel_steering_joint);

  read(gazebo_interface.rear_left_wheel_spinning_joint,
       hardware_interface.rear_left_wheel_spinning_joint);
  read(gazebo_interface.rear_right_wheel_spinning_joint,
       hardware_interface.rear_right_wheel_spinning_joint);

  read(gazebo_interface.front_wheel_spinning_joint,
       hardware_interface.front_wheel_spinning_joint_feedback);

}


}
