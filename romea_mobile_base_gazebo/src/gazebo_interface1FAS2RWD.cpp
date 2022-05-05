#include "romea_mobile_base_gazebo/gazebo_interface1FAS2RWD.hpp"
#include <romea_mobile_base_hardware/hardware_info.hpp>

#include <romea_core_mobile_base/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface1FAS2RWD::GazeboInterface1FAS2RWD(gazebo::physics::ModelPtr parent_model,
                                                 const hardware_interface::HardwareInfo & hardware_info,
                                                 const std::string & command_interface_type):
  front_axle_steering_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2RWD::FRONT_AXLE_STEERING_JOINT_ID]),
  front_left_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2RWD::FRONT_LEFT_WHEEL_STEERING_JOINT_ID]),
  front_right_wheel_steering_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2RWD::FRONT_RIGHT_WHEEL_STEERING_JOINT_ID]),
  front_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2RWD::FRONT_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  front_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2RWD::FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2RWD::REAR_LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  rear_right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface1FAS2RWD::REAR_RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  wheelbase(get_parameter<double>(hardware_info,"wheelbase")),
  front_track(get_parameter<double>(hardware_info,"front_track")),
  front_hub_carrier_offset(get_parameter<double>(hardware_info,"front_hub_carrier_offset"))
{

}


//-----------------------------------------------------------------------------
void write(const HardwareInterface1FAS2RWD & hardware_interface,
           const double & front_left_angle_command,
           const double & front_right_angle_command,
           const double & front_left_speed_command,
           const double & front_right_speed_command,
           GazeboInterface1FAS2RWD & gazebo_interface)
{

  write(hardware_interface.front_axle_steering_joint,
        gazebo_interface.front_axle_steering_joint);

  gazebo_interface.front_left_wheel_steering_joint.
      setCommand(front_left_angle_command);
  gazebo_interface.front_right_wheel_steering_joint.
      setCommand(front_right_angle_command);

  gazebo_interface.front_left_wheel_spinning_joint.
      setCommand(front_left_speed_command);
  gazebo_interface.front_right_wheel_spinning_joint.
      setCommand(front_right_speed_command);

  write(hardware_interface.rear_left_wheel_spinning_joint,
        gazebo_interface.rear_left_wheel_spinning_joint);
  write(hardware_interface.rear_right_wheel_spinning_joint,
        gazebo_interface.rear_right_wheel_spinning_joint);

}

//-----------------------------------------------------------------------------
void write(const HardwareInterface1FAS2RWD & hardware_interface,
           GazeboInterface1FAS2RWD & gazebo_interface)
{
  const double & wheelbase = gazebo_interface.wheelbase;
  const double & front_track = gazebo_interface.front_track;
  const double & front_hub_carrier_offset = gazebo_interface.front_hub_carrier_offset;

  double tan_axle_steering_command =
      std::tan(hardware_interface.front_axle_steering_joint.command.get());

  double intantaneous_curvature_command = OneAxleSteeringKinematic::
      computeInstantaneousCurvature(tan_axle_steering_command,wheelbase);

  double front_left_angle_command = TwoWheelSteeringKinematic::
      computeLeftWheelAngle(tan_axle_steering_command,
                            intantaneous_curvature_command,
                            front_track/2.);

  double front_right_angle_command = TwoWheelSteeringKinematic::
      computeRightWheelAngle(tan_axle_steering_command,
                             intantaneous_curvature_command,
                             front_track/2.);

  double linear_speed_command =
      (hardware_interface.rear_left_wheel_spinning_joint.command.get()+
       hardware_interface.rear_right_wheel_spinning_joint.command.get())/2.0;

  double front_left_speed_command = TwoWheelSteeringKinematic::
      computeLeftWheelSpeed(linear_speed_command,
                            tan_axle_steering_command,
                            intantaneous_curvature_command,
                            front_hub_carrier_offset,
                            front_track/2.);

  double front_right_speed_command = TwoWheelSteeringKinematic::
      computeRightWheelSpeed(linear_speed_command,
                             tan_axle_steering_command,
                             intantaneous_curvature_command,
                             front_hub_carrier_offset,
                             front_track/2.);

  write(hardware_interface,
        front_left_angle_command,
        front_right_angle_command,
        front_left_speed_command,
        front_right_speed_command,
        gazebo_interface);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface1FAS2RWD & gazebo_interface,
          const double & front_axle_steering_angle_state,
          HardwareInterface1FAS2RWD & hardware_interface)
{
  hardware_interface.front_axle_steering_joint.feedback.
      set(front_axle_steering_angle_state);

  read(gazebo_interface.rear_left_wheel_spinning_joint,
       hardware_interface.rear_left_wheel_spinning_joint);
  read(gazebo_interface.rear_right_wheel_spinning_joint,
       hardware_interface.rear_right_wheel_spinning_joint);

  read(gazebo_interface.front_left_wheel_steering_joint,
       hardware_interface.front_left_wheel_steering_joint_feedback);
  read(gazebo_interface.front_right_wheel_steering_joint,
       hardware_interface.front_right_wheel_steering_joint_feedback);

  read(gazebo_interface.front_left_wheel_spinning_joint,
       hardware_interface.front_left_wheel_spinning_joint_feedback);
  read(gazebo_interface.front_right_wheel_spinning_joint,
       hardware_interface.front_right_wheel_spinning_joint_feedback);

}

//-----------------------------------------------------------------------------
void read(const GazeboInterface1FAS2RWD & gazebo_interface,
          HardwareInterface1FAS2RWD & hardware_interface)
{

  const double & wheelbase = gazebo_interface.wheelbase;
  const double & front_track = gazebo_interface.front_track;

  double front_left_wheel_steering_angle = gazebo_interface.
      front_left_wheel_spinning_joint.getFeedback().position;

  double front_right_wheel_steering_angle = gazebo_interface.
      front_right_wheel_spinning_joint.getFeedback().position;

  double front_axle_steering_angle_state = TwoWheelSteeringKinematic::
      computeSteeringAngle(front_left_wheel_steering_angle,
                           front_right_wheel_steering_angle,
                           wheelbase,front_track);

  read(gazebo_interface,
       front_axle_steering_angle_state,
       hardware_interface);
}

}
