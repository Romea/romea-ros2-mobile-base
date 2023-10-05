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


// std
#include <memory>
#include <string>
#include <vector>

// local
#include "romea_mobile_base_gazebo/gazebo_system_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
GazeboSystemInterface<GazeboInterface, SimulationInterface>::GazeboSystemInterface()
: nh_(),
  parent_model_(),
  gazebo_interface_(nullptr),
  simulation_interface_(nullptr)
{
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
bool GazeboSystemInterface<GazeboInterface, SimulationInterface>::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  gazebo::physics::ModelPtr parent_model,
  const hardware_interface::HardwareInfo & hardware_info,
  sdf::ElementPtr sdf)
{
  nh_ = model_nh;
  parent_model_ = parent_model;

  // RCLCPP_ERROR(this->nh_->get_logger(), "initSim");

  return check_physics_engine_configuration_() &&
         init_gazebo_interfaces_(hardware_info) &&
         init_hardware_interfaces_(hardware_info);
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
bool GazeboSystemInterface<GazeboInterface,
  SimulationInterface>::check_physics_engine_configuration_()
{
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(this->nh_->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  // RCLCPP_ERROR(this->nh_->get_logger(), "check_physics_engine_configuration_ OK");

  return true;
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GazeboSystemInterface<GazeboInterface, SimulationInterface>::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GazeboSystemInterface<GazeboInterface, SimulationInterface>::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GazeboSystemInterface<GazeboInterface, SimulationInterface>::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
std::vector<hardware_interface::StateInterface>
GazeboSystemInterface<GazeboInterface, SimulationInterface>::export_state_interfaces()
{
  return simulation_interface_->export_state_interfaces();
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
std::vector<hardware_interface::CommandInterface>
GazeboSystemInterface<GazeboInterface, SimulationInterface>::export_command_interfaces()
{
  return simulation_interface_->export_command_interfaces();
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
bool GazeboSystemInterface<GazeboInterface, SimulationInterface>::init_gazebo_interfaces_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    gazebo_interface_ = std::make_unique<GazeboInterface>(parent_model_, hardware_info, "velocity");
    // RCLCPP_ERROR(this->nh_->get_logger(), "init_gazebo_interfaces_ OK");
    return true;
  } catch (std::runtime_error & e) {
    // RCLCPP_ERROR(this->nh_->get_logger(), "init_gazebo_interfaces_ not OK");
    RCLCPP_ERROR_STREAM(nh_->get_logger(), e.what());
    return false;
  }
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
bool GazeboSystemInterface<GazeboInterface, SimulationInterface>::init_hardware_interfaces_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    simulation_interface_ = std::make_unique<SimulationInterface>(hardware_info, "velocity");
    // RCLCPP_ERROR(this->nh_->get_logger(), "init_hardware_interfaces_ OK");
    return true;
  } catch (std::runtime_error & e) {
    // RCLCPP_ERROR(this->nh_->get_logger(), "init_hardware_interfaces_ not OK");
    RCLCPP_ERROR_STREAM(nh_->get_logger(), e.what());
    return false;
  }
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type GazeboSystemInterface<GazeboInterface, SimulationInterface>::read()
#else
hardware_interface::return_type GazeboSystemInterface<GazeboInterface, SimulationInterface>::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
#endif
{
  simulation_interface_->set_state(gazebo_interface_->get_state());
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type GazeboSystemInterface<GazeboInterface, SimulationInterface>::write()
#else
hardware_interface::return_type GazeboSystemInterface<GazeboInterface, SimulationInterface>::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
#endif
{
  gazebo_interface_->set_command(simulation_interface_->get_command());
  return hardware_interface::return_type::OK;
}

template class GazeboSystemInterface<GazeboInterface1FASxxx, SimulationInterface1FAS2FWD>;
template class GazeboSystemInterface<GazeboInterface1FASxxx, SimulationInterface1FAS2RWD>;
template class GazeboSystemInterface<GazeboInterface2ASxxx, SimulationInterface2AS4WD>;
template class GazeboSystemInterface<GazeboInterface2FWSxxx, SimulationInterface2FWS2FWD>;
template class GazeboSystemInterface<GazeboInterface2FWSxxx, SimulationInterface2FWS2RWD>;
template class GazeboSystemInterface<GazeboInterface2FWSxxx, SimulationInterface2FWS4WD>;
template class GazeboSystemInterface<GazeboInterface2TD, SimulationInterface2TD>;
template class GazeboSystemInterface<GazeboInterface2THD, SimulationInterface2THD>;
template class GazeboSystemInterface<GazeboInterface2TTD, SimulationInterface2TTD>;
template class GazeboSystemInterface<GazeboInterface2WD, SimulationInterface2WD>;
template class GazeboSystemInterface<GazeboInterface4WD, SimulationInterface4WD>;
template class GazeboSystemInterface<GazeboInterface4WS4WD, SimulationInterface4WS4WD>;

}  // namespace romea

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  romea::GazeboSystemInterface4WD,
  gazebo_ros2_control::GazeboSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::GazeboSystemInterface4WS4WD,
  gazebo_ros2_control::GazeboSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::GazeboSystemInterface2AS4WD,
  gazebo_ros2_control::GazeboSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::GazeboSystemInterface2FWS4WD,
  gazebo_ros2_control::GazeboSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::GazeboSystemInterface2FWS2RWD,
  gazebo_ros2_control::GazeboSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::GazeboSystemInterface2THD,
  gazebo_ros2_control::GazeboSystemInterface)
