// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// string
#include <string>

// local
#include "romea_mobile_base_gazebo/spinning_joint_gazebo_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SpinningJointGazeboInterface::SpinningJointGazeboInterface(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & command_interface_type)
{
  if (!command_interface_type.compare(hardware_interface::HW_IF_VELOCITY)) {
    control_type = core::RotationalMotionControlType::VELOCITY;
  } else if (!command_interface_type.compare(hardware_interface::HW_IF_EFFORT)) {
    control_type = core::RotationalMotionControlType::TORQUE;
  } else {
    // throw error
  }

  // std::cout << " spinning joint_info.name ";
  // std::cout << joint_info.name << std::endl;
  sim_joint_ = parent_model->GetJoint(joint_info.name);

  if (sim_joint_ == nullptr) {
    std::stringstream msg;
    msg << " Joint called ";
    msg << joint_info.name;
    msg << " cannot be get by spinnig joint gazebo interface";
    throw std::runtime_error(msg.str());
  }
}

//-----------------------------------------------------------------------------
void SpinningJointGazeboInterface::set_command(const double & command)
{
  if (control_type == core::RotationalMotionControlType::VELOCITY) {
    sim_joint_->SetVelocity(0, command);
  } else {
    sim_joint_->SetForce(0, command);
  }
}

//-----------------------------------------------------------------------------
core::RotationalMotionState SpinningJointGazeboInterface::get_state() const
{
  core::RotationalMotionState state;
  state.position = sim_joint_->Position(0);
  state.velocity = sim_joint_->GetVelocity(0);
  state.torque = sim_joint_->GetForce(0);
  return state;
}

}  // namespace ros2
}  // namespace romea
