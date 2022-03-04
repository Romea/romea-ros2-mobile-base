#include "romea_mobile_base_gazebo/gazebo_interface1FAS2FWD.hpp"
#include <romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp>
#include <romea_core_mobile_base/kinematic/skid_steering/SkidSteeringKinematic.hpp>


namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface1FAS2FWD::GazeboInterface1FAS2FWD(gazebo::physics::ModelPtr parent_model,
                                                 const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & command_interface_type):
  front_axle_steering_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_AXLE_STEERING_JOINT_ID]),
  front_left_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2FWD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  wheelbase(0),
  front_track(0),
  rear_track(0)
{

}


//-----------------------------------------------------------------------------
void write(const HardwareInterface1FAS2FWD & hardware_interface,
           const double & front_left_wheel_angle_command,
           const double & front_right_wheel_angle_command,
           const double & rear_left_wheel_speed_command,
           const double & rear_right_wheel_speed_command,
           GazeboInterface1FAS2FWD & gazebo_interface)
{

  write(hardware_interface.front_axle_steering_joint,
        gazebo_interface.front_axle_steering_joint);

  gazebo_interface.front_left_wheel_steering_joint.
      setCommand(front_left_wheel_angle_command);
  gazebo_interface.front_right_wheel_steering_joint.
      setCommand(front_right_wheel_angle_command);

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
void write(const HardwareInterface1FAS2FWD & hardware_interface,
           GazeboInterface1FAS2FWD & gazebo_interface)
{
  const double & wheelbase = gazebo_interface.wheelbase;
  const double & front_track = gazebo_interface.front_track;
  const double & rear_track = gazebo_interface.rear_track;

  double front_left_wheel_speed_command =
      hardware_interface.front_left_wheel_spinning_joint.command.get();
  double front_right_wheel_speed_command =
      hardware_interface.front_right_wheel_spinning_joint.command.get();

  double tan_axle_steering_command =
      std::tan(hardware_interface.front_axle_steering_joint.command.get());

  double intantaneous_curvature_command = OneAxleSteeringKinematic::
      computeInstantaneousCurvature(tan_axle_steering_command,wheelbase);


  double front_left_wheel_angle_command = TwoWheelSteeringKinematic::
      computeLeftWheelAngle(tan_axle_steering_command,
                            intantaneous_curvature_command,
                            front_track/2.);

  double front_right_wheel_angle_command = TwoWheelSteeringKinematic::
      computeRightWheelAngle(tan_axle_steering_command,
                             intantaneous_curvature_command,
                             front_track/2.);

  double linear_speed_command =
      (front_left_wheel_speed_command*std::cos(front_left_wheel_angle_command)+
       front_right_wheel_speed_command*std::cos(front_right_wheel_angle_command))/2.;

  double angular_speed_command = intantaneous_curvature_command*linear_speed_command;

  double rear_left_wheel_speed_command = SkidSteeringKinematic::
      computeLeftWheelSpeed(linear_speed_command,angular_speed_command,rear_track);

  double rear_right_wheel_speed_command = SkidSteeringKinematic::
      computeRightWheelSpeed(linear_speed_command,angular_speed_command,rear_track);


    write(hardware_interface,
          front_left_wheel_angle_command,
          front_right_wheel_angle_command,
          rear_left_wheel_speed_command,
          rear_right_wheel_speed_command,
          gazebo_interface);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface1FAS2FWD & gazebo_interface,
          const double & front_axle_steering_angle_command,
          HardwareInterface1FAS2FWD & hardware_interface)
{
  //  hardware_interface.front_axle_steering_joint.feedback.
  //      set(front_axle_steering_angle_command);

  //  read(gazebo_interface.rear_left_wheel_spinning_joint,
  //       hardware_interface.rear_left_wheel_spinning_joint);
  //  read(gazebo_interface.rear_right_wheel_spinning_joint,
  //       hardware_interface.rear_right_wheel_spinning_joint);

  //  read(gazebo_interface.front_left_wheel_steering_joint,
  //       hardware_interface.front_left_wheel_steering_joint_feedback);
  //  read(gazebo_interface.front_right_wheel_steering_joint,
  //       hardware_interface.front_right_wheel_steering_joint_feedback);

  //  read(gazebo_interface.front_left_wheel_spinning_joint,
  //       hardware_interface.front_left_wheel_spinning_joint_feedback);
  //  read(gazebo_interface.front_right_wheel_spinning_joint,
  //       hardware_interface.front_right_wheel_spinning_joint_feedback);

}

//-----------------------------------------------------------------------------
void read(const GazeboInterface1FAS2FWD & gazebo_interface,
          HardwareInterface1FAS2FWD & hardware_interface)
{

  //  const double & wheelbase = gazebo_interface.wheelbase;
  //  const double & front_track = gazebo_interface.front_track;

  //  double front_left_wheel_steering_angle = gazebo_interface.
  //      front_left_wheel_spinning_joint.getFeedback().position;

  //  double front_right_wheel_steering_angle = gazebo_interface.
  //      front_right_wheel_spinning_joint.getFeedback().position;

  //  double front_axle_steering_angle_command = TwoWheelSteeringKinematic::
  //      computeSteeringAngle(front_left_wheel_steering_angle,
  //                           front_right_wheel_steering_angle,
  //                           wheelbase,front_track);

  //  read(gazebo_interface,
  //       front_axle_steering_angle_command,
  //       hardware_interface);
}

}