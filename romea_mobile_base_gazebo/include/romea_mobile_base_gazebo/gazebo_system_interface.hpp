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


#ifndef ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_SYSTEM_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_SYSTEM_INTERFACE_HPP_

// std
#include <memory>
#include <vector>

// ros
#include "gazebo_ros2_control/gazebo_system_interface.hpp"

// romea
#include "romea_common_utils/ros_versions.hpp"
#include "romea_mobile_base_simulation/simulation_interface1FAS2FWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface1FAS2RWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2AS4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWS2FWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWS2RWD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2FWS4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2TD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2THD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2TTD.hpp"
#include "romea_mobile_base_simulation/simulation_interface2WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface4WD.hpp"
#include "romea_mobile_base_simulation/simulation_interface4WS4WD.hpp"


// local
#include "romea_mobile_base_gazebo/gazebo_interface1FASxxx.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2ASxxx.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2FWSxxx.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2TD.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2THD.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2TTD.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface2WD.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface4WD.hpp"
#include "romea_mobile_base_gazebo/gazebo_interface4WS4WD.hpp"


namespace romea
{
namespace ros2
{

template<typename GazeboInterface, typename SimulationInterface>
class GazeboSystemInterface : public gazebo_ros2_control::GazeboSystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  GazeboSystemInterface();

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf) override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

#if ROS_DISTRO == ROS_GALACTIC
  virtual hardware_interface::return_type read();

  virtual hardware_interface::return_type write();
#else
  virtual hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period);

  virtual hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period);
#endif

private:
  bool check_physics_engine_configuration_();

  bool init_gazebo_interfaces_(const hardware_interface::HardwareInfo & hardware_info);

  bool init_hardware_interfaces_(const hardware_interface::HardwareInfo & hardware_info);

private:
  rclcpp::Node::SharedPtr nh_;
  gazebo::physics::ModelPtr parent_model_;
  std::unique_ptr<GazeboInterface> gazebo_interface_;
  std::unique_ptr<SimulationInterface> simulation_interface_;
};

using GazeboSystemInterface1FAS2FWD =
  GazeboSystemInterface<GazeboInterface1FASxxx, SimulationInterface1FAS2FWD>;
using GazeboSystemInterface1FAS2RWD =
  GazeboSystemInterface<GazeboInterface1FASxxx, SimulationInterface1FAS2RWD>;
using GazeboSystemInterface2AS4WD =
  GazeboSystemInterface<GazeboInterface2ASxxx, SimulationInterface2AS4WD>;
using GazeboSystemInterface2FWS2FWD =
  GazeboSystemInterface<GazeboInterface2FWSxxx, SimulationInterface2FWS2FWD>;
using GazeboSystemInterface2FWS2RWD =
  GazeboSystemInterface<GazeboInterface2FWSxxx, SimulationInterface2FWS2RWD>;
using GazeboSystemInterface2FWS4WD =
  GazeboSystemInterface<GazeboInterface2FWSxxx, SimulationInterface2FWS4WD>;
using GazeboSystemInterface2TD =
  GazeboSystemInterface<GazeboInterface2TD, SimulationInterface2TD>;
using GazeboSystemInterface2THD =
  GazeboSystemInterface<GazeboInterface2THD, SimulationInterface2THD>;
using GazeboSystemInterface2TTD =
  GazeboSystemInterface<GazeboInterface2TTD, SimulationInterface2TTD>;
using GazeboSystemInterface2WD =
  GazeboSystemInterface<GazeboInterface2WD, SimulationInterface2WD>;
using GazeboSystemInterface4WD =
  GazeboSystemInterface<GazeboInterface4WD, SimulationInterface4WD>;
using GazeboSystemInterface4WS4WD =
  GazeboSystemInterface<GazeboInterface4WS4WD, SimulationInterface4WS4WD>;

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_GAZEBO__GAZEBO_SYSTEM_INTERFACE_HPP_
