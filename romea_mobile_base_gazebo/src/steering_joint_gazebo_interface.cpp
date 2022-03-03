#include "romea_mobile_base_gazebo_interfaces/steering_joint_gazebo_interface.hpp"
#include "romea_mobile_base_gazebo_interfaces/hardware_interface_info.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
SteeringJointGazeboInterface::SteeringJointGazeboInterface(const std::string & joint_name,
                                                           gazebo::physics::JointPtr sim_joint):
  JointGazeboInterface(joint_name,sim_joint)
{

}

//-----------------------------------------------------------------------------
void SteeringJointGazeboInterface::read()
{
  measurement_=sim_joint_->Position(0);
}

//-----------------------------------------------------------------------------
void SteeringJointGazeboInterface::write()
{
  sim_joint_->SetPosition(0,command_);
}

//-----------------------------------------------------------------------------
hardware_interface::CommandInterface SteeringJointGazeboInterface::exportCommandInterface()
{
  return hardware_interface::CommandInterface(joint_name_,
                                              hardware_interface::HW_IF_POSITION,
                                              & command_);
}

//-----------------------------------------------------------------------------
hardware_interface::StateInterface SteeringJointGazeboInterface::exportStateInterface()
{
  return hardware_interface::StateInterface(joint_name_,
                                            hardware_interface::HW_IF_POSITION,
                                            & measurement_);
}

//-----------------------------------------------------------------------------
std::unique_ptr<SteeringJointGazeboInterface>
makeSteeringJointHarwareInterface(gazebo::physics::ModelPtr parent_model,
                                  const hardware_interface::ComponentInfo & joint_info)
{
  auto command_interface_info = get_command_interface_info(
        joint_info,hardware_interface::HW_IF_VELOCITY);
  auto state_interface_info = get_state_interface_info(
        joint_info,hardware_interface::HW_IF_VELOCITY);
  //TODO extract info
  return std::make_unique<SteeringJointGazeboInterface>(
        joint_info.name,parent_model->GetJoint(joint_info.name));

}


}
