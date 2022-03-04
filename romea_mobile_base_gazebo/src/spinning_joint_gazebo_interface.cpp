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
void read(const SpinningJointGazeboInterface & gazebo_joint,
          SpinningJointHardwareInterface::Feedback & hardware_joint_feedback)
{
  auto feedback = gazebo_joint.getFeedback();
  hardware_joint_feedback.position.set(feedback.position);
  hardware_joint_feedback.velocity.set(feedback.velocity);
  hardware_joint_feedback.torque.set(feedback.effort);
}

}
