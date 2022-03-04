#include "romea_mobile_base_gazebo/gazebo_interface2FWS2RWD.hpp"
#include <romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2FWS2RWD::GazeboInterface2FWS2RWD(gazebo::physics::ModelPtr parent_model,
                                                 const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & command_interface_type):
  front_left_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2RWD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2RWD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2RWD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2RWD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2RWD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2RWD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  wheelbase(0),
  front_track(0),
  front_hub_carrier_offset(0)
{

}


//-----------------------------------------------------------------------------
void write(const HardwareInterface2FWS2RWD & hardware_interface,
           const double & front_left_wheel_speed_command,
           const double & front_right_wheel_speed_command,
           GazeboInterface2FWS2RWD & gazebo_interface)
{

  write(hardware_interface.front_left_wheel_steering_joint,
        gazebo_interface.front_left_wheel_steering_joint);
  write(hardware_interface.front_right_wheel_steering_joint,
        gazebo_interface.front_right_wheel_steering_joint);

  gazebo_interface.front_left_wheel_spinning_joint.
      setCommand(front_left_wheel_speed_command);
  gazebo_interface.front_right_wheel_spinning_joint.
      setCommand(front_right_wheel_speed_command);

  write(hardware_interface.rear_left_wheel_spinning_joint,
        gazebo_interface.rear_left_wheel_spinning_joint);
  write(hardware_interface.rear_right_wheel_spinning_joint,
        gazebo_interface.rear_right_wheel_spinning_joint);

}

//-----------------------------------------------------------------------------
void write(const HardwareInterface2FWS2RWD & hardware_interface,
           GazeboInterface2FWS2RWD & gazebo_interface)
{

  const double & wheelbase = gazebo_interface.wheelbase;
  const double & front_track = gazebo_interface.front_track;
  const double & front_hub_carrier_offset = gazebo_interface.front_hub_carrier_offset;

  double linear_speed_command =
      (hardware_interface.rear_left_wheel_spinning_joint.command.get()+
       hardware_interface.rear_right_wheel_spinning_joint.command.get())/2.0;

  double front_left_wheel_angle_command =
      hardware_interface.front_left_wheel_steering_joint.command.get();
  double front_right_wheel_angle_command =
      hardware_interface.front_right_wheel_steering_joint.command.get();

  double instantaneous_curvature_command = TwoWheelSteeringKinematic::
      computeInstantaneousCurvature(front_right_wheel_angle_command,
                                    front_left_wheel_angle_command,
                                    wheelbase,front_track);

  double front_left_wheel_speed_command= TwoWheelSteeringKinematic::
      computeLeftWheelSpeed(linear_speed_command,
                            instantaneous_curvature_command*wheelbase,
                            instantaneous_curvature_command,
                            front_hub_carrier_offset,
                            front_track/2.0);

  double front_right_wheel_speed_command= TwoWheelSteeringKinematic::
      computeRightWheelSpeed(linear_speed_command,
                             instantaneous_curvature_command*wheelbase,
                             instantaneous_curvature_command,
                             front_hub_carrier_offset,
                             front_track/2.0);


  write(hardware_interface,
        front_left_wheel_speed_command,
        front_right_wheel_speed_command,
        gazebo_interface);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface2FWS2RWD & gazebo_interface,
          HardwareInterface2FWS2RWD & hardware_interface)
{
  read(gazebo_interface.front_left_wheel_steering_joint,
       hardware_interface.front_left_wheel_steering_joint);
  read(gazebo_interface.front_right_wheel_steering_joint,
       hardware_interface.front_right_wheel_steering_joint);

  read(gazebo_interface.rear_left_wheel_spinning_joint,
       hardware_interface.rear_left_wheel_spinning_joint);
  read(gazebo_interface.rear_right_wheel_spinning_joint,
       hardware_interface.rear_right_wheel_spinning_joint);

  read(gazebo_interface.front_left_wheel_spinning_joint,
       hardware_interface.front_left_wheel_spinning_joint_feedback);
  read(gazebo_interface.front_right_wheel_spinning_joint,
       hardware_interface.front_right_wheel_spinning_joint_feedback);

}


}
