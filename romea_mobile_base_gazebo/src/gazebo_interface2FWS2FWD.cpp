#include "romea_mobile_base_gazebo/gazebo_interface2FWS2FWD.hpp"
#include <romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2FWS2FWD::GazeboInterface2FWS2FWD(gazebo::physics::ModelPtr parent_model,
                                                 const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & command_interface_type):
  front_left_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2FWD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2FWD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2FWD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2FWD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2FWD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2FWS2FWD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  wheelbase(0),
  front_track(0),
  rear_track(0)
{

}


//-----------------------------------------------------------------------------
void write(const HardwareInterface2FWS2FWD & hardware_interface,
           const double & rear_left_wheel_speed_command,
           const double & rear_right_wheel_speed_command,
           GazeboInterface2FWS2FWD & gazebo_interface)
{
  write(hardware_interface.front_left_wheel_steering_joint,
        gazebo_interface.front_left_wheel_steering_joint);
  write(hardware_interface.front_right_wheel_steering_joint,
        gazebo_interface.front_right_wheel_steering_joint);

  write(hardware_interface.front_left_wheel_spinning_joint,
        gazebo_interface.front_left_wheel_spinning_joint);
  write(hardware_interface.front_right_wheel_spinning_joint,
        gazebo_interface.front_right_wheel_spinning_joint);

  gazebo_interface.rear_left_wheel_spinning_joint.
      setCommand(rear_left_wheel_speed_command);
  gazebo_interface.rear_right_wheel_spinning_joint.
      setCommand(rear_right_wheel_speed_command);

}

//-----------------------------------------------------------------------------
void write(const HardwareInterface2FWS2FWD & hardware_interface,
           GazeboInterface2FWS2FWD & gazebo_interface)
{
  const double & wheelbase = gazebo_interface.wheelbase;
  const double & front_track = gazebo_interface.front_track;
  const double & rear_track = gazebo_interface.rear_track;

  double front_left_wheel_speed_command =
      hardware_interface.front_left_wheel_spinning_joint.command.get();
  double front_right_wheel_speed_command =
      hardware_interface.front_right_wheel_spinning_joint.command.get();

  double front_left_wheel_angle_command =
      hardware_interface.front_left_wheel_steering_joint.command.get();
  double front_right_wheel_angle_command =
      hardware_interface.front_right_wheel_steering_joint.command.get();

  double linear_speed_command =
      (front_left_wheel_speed_command*std::cos(front_left_wheel_angle_command)+
       front_right_wheel_speed_command*std::cos(front_right_wheel_angle_command))/2.0;

  double angular_speed_command = TwoWheelSteeringKinematic::
      computeInstantaneousCurvature(front_right_wheel_angle_command,
                                    front_left_wheel_angle_command,
                                    wheelbase,front_track)*linear_speed_command;

  double rear_left_wheel_speed_command = SkidSteeringKinematic::
      computeLeftWheelSpeed(linear_speed_command,angular_speed_command,rear_track);

  double rear_right_wheel_speed_command = SkidSteeringKinematic::
      computeRightWheelSpeed(linear_speed_command,angular_speed_command,rear_track);


   write(hardware_interface,
         rear_left_wheel_speed_command,
         rear_right_wheel_speed_command,
         gazebo_interface);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface2FWS2FWD & gazebo_interface,
          HardwareInterface2FWS2FWD & hardware_interface)
{
  read(gazebo_interface.front_left_wheel_steering_joint,
       hardware_interface.front_left_wheel_steering_joint);
  read(gazebo_interface.front_right_wheel_steering_joint,
       hardware_interface.front_right_wheel_steering_joint);

  read(gazebo_interface.front_left_wheel_spinning_joint,
       hardware_interface.front_left_wheel_spinning_joint);
  read(gazebo_interface.front_right_wheel_spinning_joint,
       hardware_interface.front_right_wheel_spinning_joint);

  read(gazebo_interface.rear_left_wheel_spinning_joint,
       hardware_interface.rear_left_wheel_spinning_joint_feedback);
  read(gazebo_interface.rear_right_wheel_spinning_joint,
       hardware_interface.rear_right_wheel_spinning_joint_feedback);

}


}
