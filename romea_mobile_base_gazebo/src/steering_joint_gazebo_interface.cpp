#include "romea_mobile_base_gazebo/steering_joint_gazebo_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SteeringJointGazeboInterface::SteeringJointGazeboInterface(
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::ComponentInfo & joint_info)
{
  sim_joint_ = parent_model->GetJoint(joint_info.name);
}

//-----------------------------------------------------------------------------
void SteeringJointGazeboInterface::set_command(const double & command)
{
  sim_joint_->SetPosition(0,command,true);
}

//-----------------------------------------------------------------------------
double SteeringJointGazeboInterface::get_state() const
{
  return sim_joint_->Position(0);
}

////-----------------------------------------------------------------------------
//void write(const SteeringJointHardwareInterface & hardware_joint,
//           SteeringJointGazeboInterface & gazebo_joint)
//{
//  gazebo_joint.setCommand(hardware_joint.command.get());
//}

////-----------------------------------------------------------------------------
//void read(const SteeringJointGazeboInterface & gazebo_joint,
//          SteeringJointHardwareInterface & hardware_joint)
//{
//  read(gazebo_joint,hardware_joint.feedback);
//}

////-----------------------------------------------------------------------------
//void read(const SteeringJointGazeboInterface & gazebo_joint,
//          SteeringJointHardwareInterface::Feedback & hardware_joint_feedback)
//{
//  hardware_joint_feedback.set(gazebo_joint.getFeedback());
//}

}
