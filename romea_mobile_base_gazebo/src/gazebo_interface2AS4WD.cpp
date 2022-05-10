#include "romea_mobile_base_gazebo/gazebo_interface2AS4WD.hpp"
#include <romea_mobile_base_hardware/hardware_info.hpp>

#include <romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp>


namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2AS4WD::GazeboInterface2AS4WD(gazebo::physics::ModelPtr parent_model,
                                             const hardware_interface::HardwareInfo & hardware_info,
                                             const std::string & command_interface_type):
  front_axle_steering_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_AXLE_STEERING_JOINT_ID)),
  rear_axle_steering_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_AXLE_STEERING_JOINT_ID)),
  front_left_wheel_steering_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID)),
  front_right_wheel_steering_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID)),
  rear_left_wheel_steering_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_LEFT_WHEEL_STEERING_JOINT_ID)),
  rear_right_wheel_steering_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_RIGHT_WHEEL_STEERING_JOINT_ID)),
  front_left_wheel_spinning_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID),command_interface_type),
  front_right_wheel_spinning_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID),command_interface_type),
  rear_left_wheel_spinning_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID),command_interface_type),
  rear_right_wheel_spinning_joint(parent_model,hardware_info.joints.at(HardwareInterface2AS4WD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID),command_interface_type),
  wheelbase(get_parameter<double>(hardware_info,"wheelbase")),
  front_track(get_parameter<double>(hardware_info,"front_track")),
  rear_track(get_parameter<double>(hardware_info,"rear_track"))
{

}

//-----------------------------------------------------------------------------
void write(const HardwareInterface2AS4WD & hardware_interface,
           const double & front_left_angle_command,
           const double & front_right_angle_command,
           const double & rear_left_angle_command,
           const double & rear_right_angle_command,
           GazeboInterface2AS4WD & gazebo_interface)
{

  write(hardware_interface.front_axle_steering_joint,
        gazebo_interface.front_axle_steering_joint);
  write(hardware_interface.rear_axle_steering_joint,
        gazebo_interface.rear_axle_steering_joint);

  gazebo_interface.front_left_wheel_steering_joint.
      setCommand(front_left_angle_command);
  gazebo_interface.front_right_wheel_steering_joint.
      setCommand(front_right_angle_command);

  gazebo_interface.rear_left_wheel_steering_joint.
      setCommand(rear_left_angle_command);
  gazebo_interface.rear_right_wheel_steering_joint.
      setCommand(rear_right_angle_command);

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
void write(const HardwareInterface2AS4WD & hardware_interface,
           GazeboInterface2AS4WD & gazebo_interface)
{
  const double & wheelbase = gazebo_interface.wheelbase;
  const double & front_track = gazebo_interface.front_track;
  const double & rear_track = gazebo_interface.rear_track;

  double tan_front_axle_steering_command =
      std::tan(hardware_interface.front_axle_steering_joint.command.get());

  double tan_rear_axle_steering_command =
      std::tan(-hardware_interface.rear_axle_steering_joint.command.get());

  double front_intantaneous_curvature_command = OneAxleSteeringKinematic::
      computeInstantaneousCurvature(tan_front_axle_steering_command,wheelbase);

  double rear_intantaneous_curvature_command = OneAxleSteeringKinematic::
      computeInstantaneousCurvature(tan_rear_axle_steering_command,wheelbase);

  double front_left_angle_command = TwoWheelSteeringKinematic::
      computeLeftWheelAngle(tan_front_axle_steering_command,
                            front_intantaneous_curvature_command,
                            front_track/2.);

  double front_right_angle_command = TwoWheelSteeringKinematic::
      computeRightWheelAngle(tan_front_axle_steering_command,
                             front_intantaneous_curvature_command,
                             front_track/2.);

  double rear_left_angle_command = -TwoWheelSteeringKinematic::
      computeLeftWheelAngle(tan_rear_axle_steering_command,
                            rear_intantaneous_curvature_command,
                            rear_track/2.);


  double rear_right_angle_command = -TwoWheelSteeringKinematic::
      computeRightWheelAngle(tan_rear_axle_steering_command,
                             rear_intantaneous_curvature_command,
                             rear_track/2.);

//  std::cout << " \n\n" << std::endl;
//  std::cout << " front_steering_angle_command " << hardware_interface.front_axle_steering_joint.command.get()<< std::endl;
//  std::cout << " rear_steering_angle_command " << hardware_interface.rear_axle_steering_joint.command.get()<< std::endl;
//  std::cout << " front_left_angle_command " << front_left_angle_command<< std::endl;
//  std::cout << " front_right_angle_command " << front_right_angle_command<< std::endl;
//  std::cout << " rear_left_angle_command " << rear_left_angle_command<< std::endl;
//  std::cout << " rear_right_angle_command " << rear_right_angle_command<< std::endl;


  write(hardware_interface,
        front_left_angle_command,
        front_right_angle_command,
        rear_left_angle_command,
        rear_right_angle_command,
        gazebo_interface);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface2AS4WD & gazebo_interface,
          const double & front_axle_steering_angle_state,
          const double & rear_axle_steering_angle_state,
          HardwareInterface2AS4WD & hardware_interface)
{
  hardware_interface.front_axle_steering_joint.feedback.
      set(front_axle_steering_angle_state);
  hardware_interface.rear_axle_steering_joint.feedback.
      set(rear_axle_steering_angle_state);

  read(gazebo_interface.front_left_wheel_spinning_joint,
       hardware_interface.front_left_wheel_spinning_joint);
  read(gazebo_interface.front_right_wheel_spinning_joint,
       hardware_interface.front_right_wheel_spinning_joint);

  read(gazebo_interface.rear_left_wheel_spinning_joint,
       hardware_interface.rear_left_wheel_spinning_joint);
  read(gazebo_interface.rear_right_wheel_spinning_joint,
       hardware_interface.rear_right_wheel_spinning_joint);

  read(gazebo_interface.front_left_wheel_steering_joint,
       hardware_interface.front_left_wheel_steering_joint_feedback);
  read(gazebo_interface.front_right_wheel_steering_joint,
       hardware_interface.front_right_wheel_steering_joint_feedback);

  read(gazebo_interface.rear_left_wheel_steering_joint,
       hardware_interface.rear_left_wheel_steering_joint_feedback);
  read(gazebo_interface.rear_right_wheel_steering_joint,
       hardware_interface.rear_right_wheel_steering_joint_feedback);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface2AS4WD & gazebo_interface,
          HardwareInterface2AS4WD & hardware_interface)
{
  const double & wheelbase = gazebo_interface.wheelbase;
  const double & front_track = gazebo_interface.front_track;
  const double & rear_track = gazebo_interface.rear_track;

  double front_left_wheel_steering_angle = gazebo_interface.
      front_left_wheel_steering_joint.getFeedback();

  double front_right_wheel_steering_angle = gazebo_interface.
      front_right_wheel_steering_joint.getFeedback();

  double rear_left_wheel_steering_angle = gazebo_interface.
      rear_left_wheel_steering_joint.getFeedback();

  double rear_right_wheel_steering_angle = gazebo_interface.
      rear_right_wheel_steering_joint.getFeedback();

  double front_axle_steering_angle_state = TwoWheelSteeringKinematic::
      computeSteeringAngle(front_left_wheel_steering_angle,
                           front_right_wheel_steering_angle,
                           wheelbase,front_track);

  double rear_axle_steering_angle_state = TwoWheelSteeringKinematic::
      computeSteeringAngle(rear_left_wheel_steering_angle,
                           rear_right_wheel_steering_angle,
                           wheelbase,rear_track);

//  std::cout << " front_left_wheel_steering_angle " << front_left_wheel_steering_angle<< std::endl;
//  std::cout << " front_right_wheel_steering_angle " << front_right_wheel_steering_angle<< std::endl;
//  std::cout << " rear_left_wheel_steering_angle " << rear_left_wheel_steering_angle<< std::endl;
//  std::cout << " rear_right_wheel_steering_angle " << rear_right_wheel_steering_angle<< std::endl;
//  std::cout << " front_axle_steering_angle_state " << front_axle_steering_angle_state<< std::endl;
//  std::cout << " rear_axle_steering_angle_state " << rear_axle_steering_angle_state<< std::endl;

  read(gazebo_interface,
       front_axle_steering_angle_state,
       rear_axle_steering_angle_state,
       hardware_interface);

}

}


