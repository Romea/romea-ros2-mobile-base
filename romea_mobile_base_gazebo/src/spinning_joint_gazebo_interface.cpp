#include "romea_mobile_base_gazebo/spinning_joint_gazebo_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SpinningJointGazeboInterface::SpinningJointGazeboInterface(
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & command_interface_type)
{
  if(!command_interface_type.compare(hardware_interface::HW_IF_VELOCITY))
  {
    command_type=VELOCITY;
  }
  else if(!command_interface_type.compare(hardware_interface::HW_IF_EFFORT))
  {
    command_type=EFFORT;
  }
  else
  {
    //throw error
  }

  sim_joint_ = parent_model->GetJoint(joint_info.name);
}

//-----------------------------------------------------------------------------
void SpinningJointGazeboInterface::setCommand(const double & command)
{
  if(command_type==VELOCITY)
  {
    sim_joint_->SetVelocity(0,command);
  }
  else
  {
    sim_joint_->SetForce(0,command);
  }
}

//-----------------------------------------------------------------------------
SpinningJointGazeboInterface::Feedback SpinningJointGazeboInterface::getFeedback()const
{
  return {sim_joint_->Position(0),sim_joint_->GetVelocity(0),sim_joint_->GetForce(0)};
}

//-----------------------------------------------------------------------------
void write(const SpinningJointHardwareInterface & hardware_joint,
           SpinningJointGazeboInterface & gazebo_joint)
{
  gazebo_joint.setCommand(hardware_joint.command.get());
}

//-----------------------------------------------------------------------------
void read(const SpinningJointGazeboInterface & gazebo_joint,
          SpinningJointHardwareInterface & hardware_joint)
{
  read(gazebo_joint,hardware_joint.feedback);
}

//-----------------------------------------------------------------------------
void read(const SpinningJointGazeboInterface::Feedback & gazebo_joint_feed_back,
          SpinningJointHardwareInterface::Feedback & hardware_joint_feedback)
{
  hardware_joint_feedback.position.set(gazebo_joint_feed_back.position);
  hardware_joint_feedback.velocity.set(gazebo_joint_feed_back.velocity);
  hardware_joint_feedback.torque.set(gazebo_joint_feed_back.effort);
}

//-----------------------------------------------------------------------------
void read(const SpinningJointGazeboInterface & gazebo_joint,
          SpinningJointHardwareInterface::Feedback & hardware_joint_feedback)
{
  read(gazebo_joint.getFeedback(),hardware_joint_feedback);
}


SpinningJointGazeboInterface::Feedback
drive_wheel_feedback(const SpinningJointGazeboInterface & drive_wheel_spinning_joint,
                     const SpinningJointGazeboInterface & idler_wheel_spinning_joint)
{
  SpinningJointGazeboInterface::Feedback drive_wheel_feedback =
      drive_wheel_spinning_joint.getFeedback();
  SpinningJointGazeboInterface::Feedback idle_wheel_feedback =
      idler_wheel_spinning_joint.getFeedback();

  if(std::signbit(drive_wheel_feedback.velocity)==
     std::signbit(idle_wheel_feedback.velocity))
  {
    if(std::abs(idle_wheel_feedback.velocity)<
       std::abs(drive_wheel_feedback.velocity))
    {
      drive_wheel_feedback.velocity=idle_wheel_feedback.velocity;
      drive_wheel_feedback.effort=idle_wheel_feedback.effort;
    }
  }
  else
  {
    drive_wheel_feedback.velocity=0;
    drive_wheel_feedback.effort=0;
  }

  return drive_wheel_feedback;

}

SpinningJointGazeboInterface::Feedback
drive_wheel_feedback(const SpinningJointGazeboInterface & high_drive_wheel_spinning_joint,
                     const SpinningJointGazeboInterface & front_ground_idler_wheel_spinning_joint,
                     const SpinningJointGazeboInterface & rear_ground_idler_wheel_spinning_joint,
                     const double & high_wheel_radius,
                     const double & ground_wheel_radius)
{
  const double ratio = ground_wheel_radius/high_wheel_radius;

  SpinningJointGazeboInterface::Feedback high_drive_wheel_feedback=
      high_drive_wheel_spinning_joint.getFeedback();
  SpinningJointGazeboInterface::Feedback front_ground_wheel_feedback =
      front_ground_idler_wheel_spinning_joint.getFeedback();
  SpinningJointGazeboInterface::Feedback rear_ground_wheel_feedback =
      rear_ground_idler_wheel_spinning_joint.getFeedback();

  if(std::signbit(front_ground_wheel_feedback.velocity)==
     std::signbit(rear_ground_wheel_feedback.velocity))
  {
    if(std::abs(front_ground_wheel_feedback.velocity)<
       std::abs(rear_ground_wheel_feedback.velocity))
    {
      high_drive_wheel_feedback.velocity=ratio*front_ground_wheel_feedback.velocity;
      high_drive_wheel_feedback.effort=ratio*front_ground_wheel_feedback.effort;
    }
    else
    {
      high_drive_wheel_feedback.velocity=ratio*rear_ground_wheel_feedback.velocity;
      high_drive_wheel_feedback.effort=ratio*rear_ground_wheel_feedback.effort;
    }
  }
  else
  {
    high_drive_wheel_feedback.velocity=0;
    high_drive_wheel_feedback.effort=0;
  }

  return high_drive_wheel_feedback;
}

}
