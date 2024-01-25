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


#ifndef ROMEA_MOBILE_BASE_SIMULATION__GENERIC_SIMULATION_SYSTEM_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__GENERIC_SIMULATION_SYSTEM_INTERFACE_HPP_

// std
#include <vector>
#include <memory>
#include <string>

// ros
#include "hardware_interface/system_interface.hpp"

// romea
#include "romea_common_utils/ros_versions.hpp"
#include "romea_mobile_base_hardware/hardware_interface2WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface4WS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2FWS2FWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS2FWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2AS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FAS2RWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FAS2FWD.hpp"
#include "romea_mobile_base_hardware/hardware_interface1FAS4WD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2TD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2THD.hpp"
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"

namespace romea
{
namespace ros2
{

template<typename HardwareInterface>
class GenericSimulationSystemInterface : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  explicit GenericSimulationSystemInterface(
    const std::string & hardware_interface_name = "HardwareInterface");                                                                               // NOLINT

  virtual ~GenericSimulationSystemInterface() = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

protected:
  virtual hardware_interface::return_type load_interface_(
    const hardware_interface::HardwareInfo & hardware_info);

  virtual hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info);

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

  void feedback_callback_(sensor_msgs::msg::JointState::ConstSharedPtr msg);

protected:
  std::string hardware_interface_name_;
  std::unique_ptr<HardwareInterface> hardware_interface_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::mutex mutex_;
  sensor_msgs::msg::JointState command_;
  sensor_msgs::msg::JointState feedback_;
  bool has_feedback_;
};

// using GenericSimulationSystemInterface1FAS2FWD =
//   GenericSimulationSystemInterface<HardwareInterface1FAS2FWD>;
// using GenericSimulationSystemInterface1FAS2RWD =
//   GenericSimulationSystemInterface<HardwareInterface1FAS2RWD>;
// using GenericSimulationSystemInterface1FAS4WD =
//   GenericSimulationSystemInterface<HardwareInterface1FAS4WD>;
// using GenericSimulationSystemInterface2AS4WD =
// GenericSimulationSystemInterface<HardwareInterface2AS4WD>;
// using GenericSimulationSystemInterface2AS2FWD =
//   GenericSimulationSystemInterface<HardwareInterface2AS2FWD>;
// using GenericSimulationSystemInterface2AS2RWD =
//   GenericSimulationSystemInterface<HardwareInterface2AS2RWD>;
// using GenericSimulationSystemInterface2FWS2FWD =
//   GenericSimulationSystemInterface<HardwareInterface2FWS2FWD>;
// using GenericSimulationSystemInterface2FWS2RWD =
//   GenericSimulationSystemInterface<HardwareInterface2FWS2RWD>;
using GenericSimulationSystemInterface2FWS4WD =
  GenericSimulationSystemInterface<HardwareInterface2FWS4WD>;
// using GenericSimulationSystemInterface2WD =
//   GenericSimulationSystemInterface<HardwareInterface2WD>;
// using GenericSimulationSystemInterface4WD =
//   GenericSimulationSystemInterface<HardwareInterface4WD>;
using GenericSimulationSystemInterface4WS4WD =
  GenericSimulationSystemInterface<HardwareInterface4WS4WD>;
// using GenericSimulationSystemInterface2TD =
//   GenericSimulationSystemInterface<HardwareInterface2TD>;
// using GenericSimulationSystemInterface2THD =
//   GenericSimulationSystemInterface<HardwareInterface2THD>;
// using GenericSimulationSystemInterface2TTD =
//   GenericSimulationSystemInterface<HardwareInterface2TTD>;

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__GENERIC_SIMULATION_SYSTEM_INTERFACE_HPP_
