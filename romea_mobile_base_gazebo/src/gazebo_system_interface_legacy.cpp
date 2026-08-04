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
#include <map>
#include <memory>
#include <string>
#include <vector>

// local
#include "romea_mobile_base_gazebo/gazebo_system_interface_legacy.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::GazeboSystemInterfaceLegacy()
: nh_(nullptr), gazebo_interface_(nullptr), simulation_interface_(nullptr)
{
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
bool GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, gz::sim::Entity> & enable_joints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & ecm,
  unsigned int /*update_rate*/)
{
  nh_ = model_nh;
  return init_gazebo_interfaces_(ecm, enable_joints, hardware_info) &&
         init_hardware_interfaces_(hardware_info);
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::on_init(
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
GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
std::vector<hardware_interface::StateInterface>
GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::export_state_interfaces()
{
  return simulation_interface_->export_state_interfaces();
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
std::vector<hardware_interface::CommandInterface>
GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::export_command_interfaces()
{
  return simulation_interface_->export_command_interfaces();
}

// //-----------------------------------------------------------------------------
// template<typename GazeboInterface, typename SimulationInterface>
// hardware_interface::return_type
// GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::
// perform_command_mode_switch(
//     const std::vector<std::string> & /*start_interfaces*/,
//     const std::vector<std::string> & /*stop_interfaces*/)
// {
//   return hardware_interface::return_type::OK;
// }

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
bool GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::init_gazebo_interfaces_(
  sim::EntityComponentManager & ecm,
  std::map<std::string, sim::Entity> & enable_joints,
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    gazebo_interface_ =
      std::make_unique<GazeboInterface>(ecm, enable_joints, hardware_info, "velocity");
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
bool GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::init_hardware_interfaces_(
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
hardware_interface::return_type
GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  simulation_interface_->set_feedback(gazebo_interface_->get_state());
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template<typename GazeboInterface, typename SimulationInterface>
hardware_interface::return_type
GazeboSystemInterfaceLegacy<GazeboInterface, SimulationInterface>::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  gazebo_interface_->set_command(simulation_interface_->get_hardware_command());
  return hardware_interface::return_type::OK;
}

template class GazeboSystemInterfaceLegacy<GazeboInterface1FASxxx, SimulationInterface1FAS2FWD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface1FASxxx, SimulationInterface1FAS2RWD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface2ASxxx, SimulationInterface2AS4WD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface2FWSxxx, SimulationInterface2FWS2FWD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface2FWSxxx, SimulationInterface2FWS2RWD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface2FWSxxx, SimulationInterface2FWS4WD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface2TD, SimulationInterface2TD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface2THD, SimulationInterface2THD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface2TTD, SimulationInterface2TTD>;
// template class GazeboSystemInterfaceLegacy<GazeboInterface2WD, SimulationInterface2WD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface4WD, SimulationInterface4WD>;
template class GazeboSystemInterfaceLegacy<GazeboInterface4WS4WD, SimulationInterface4WS4WD>;

}  // namespace ros2
}  // namespace romea

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GazeboSystemInterface4WD, gz_ros2_control::GazeboSimSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GazeboSystemInterface4WS4WD, gz_ros2_control::GazeboSimSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GazeboSystemInterface1FAS2RWD, gz_ros2_control::GazeboSimSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GazeboSystemInterface2AS4WD, gz_ros2_control::GazeboSimSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GazeboSystemInterface2FWS4WD, gz_ros2_control::GazeboSimSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GazeboSystemInterface2FWS2RWD, gz_ros2_control::GazeboSimSystemInterface)
PLUGINLIB_EXPORT_CLASS(
  romea::ros2::GazeboSystemInterface2THD, gz_ros2_control::GazeboSimSystemInterface)
