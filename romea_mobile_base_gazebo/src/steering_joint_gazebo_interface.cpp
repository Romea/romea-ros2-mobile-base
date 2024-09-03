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


// ros
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// local
#include "romea_mobile_base_gazebo/steering_joint_gazebo_interface.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
SteeringJointGazeboInterface::SteeringJointGazeboInterface(
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::ComponentInfo & joint_info)
{
  // std::cout << " steering joint_info.name ";
  // std::cout << joint_info.name << std::endl;

  sim_joint_ = parent_model->GetJoint(joint_info.name);
  if (sim_joint_ == nullptr) {
    std::stringstream msg;
    msg << " Joint called ";
    msg << joint_info.name;
    msg << " cannot be get by steering joint gazebo interface";
    throw std::runtime_error(msg.str());
  }

  // std::cout << " steering joint_info.name ";
  // std::cout << joint_info.name << " ";
  // std::cout << int(sim_joint_.get() != nullptr) << std::endl;
}

//-----------------------------------------------------------------------------
void SteeringJointGazeboInterface::set_command(const double & command)
{
  sim_joint_->SetPosition(0, command, true);
}

//-----------------------------------------------------------------------------
double SteeringJointGazeboInterface::get_state() const
{
  return sim_joint_->Position(0);
}

}  // namespace ros2
}  // namespace romea
