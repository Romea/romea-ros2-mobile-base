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


#ifndef ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_SYSTEM_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_SYSTEM_INTERFACE_HPP_

// std
#include <vector>
#include <memory>
#include <string>

// ros
#include "hardware_interface/system_interface.hpp"

// local
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
class GenericHardwareSystemInterface : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  explicit GenericHardwareSystemInterface(
    const std::string & hardware_interface_name = "HardwareInterface");                                                                               // NOLINT

  virtual ~GenericHardwareSystemInterface() = default;

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

// using GenericHardwareSystemInterface1FAS2FWD =
//   GenericHardwareSystemInterface<HardwareInterface1FAS2FWD>;
// using GenericHardwareSystemInterface1FAS2RWD =
//   GenericHardwareSystemInterface<HardwareInterface1FAS2RWD>;
// using GenericHardwareSystemInterface1FAS4WD =
//   GenericHardwareSystemInterface<HardwareInterface1FAS4WD>;
// using GenericHardwareSystemInterface2AS4WD =
// GenericHardwareSystemInterface<HardwareInterface2AS4WD>;
// using GenericHardwareSystemInterface2AS2FWD =
//   GenericHardwareSystemInterface<HardwareInterface2AS2FWD>;
// using GenericHardwareSystemInterface2AS2RWD =
//   GenericHardwareSystemInterface<HardwareInterface2AS2RWD>;
// using GenericHardwareSystemInterface2FWS2FWD =
//   GenericHardwareSystemInterface<HardwareInterface2FWS2FWD>;
// using GenericHardwareSystemInterface2FWS2RWD =
//   GenericHardwareSystemInterface<HardwareInterface2FWS2RWD>;
using GenericHardwareSystemInterface2FWS4WD =
  GenericHardwareSystemInterface<HardwareInterface2FWS4WD>;
// using GenericHardwareSystemInterface2WD =
//   GenericHardwareSystemInterface<HardwareInterface2WD>;
// using GenericHardwareSystemInterface4WD =
//   GenericHardwareSystemInterface<HardwareInterface4WD>;
using GenericHardwareSystemInterface4WS4WD =
  GenericHardwareSystemInterface<HardwareInterface4WS4WD>;
// using GenericHardwareSystemInterface2TD =
//   GenericHardwareSystemInterface<HardwareInterface2TD>;
// using GenericHardwareSystemInterface2THD =
//   GenericHardwareSystemInterface<HardwareInterface2THD>;
// using GenericHardwareSystemInterface2TTD =
//   GenericHardwareSystemInterface<HardwareInterface2TTD>;

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_SYSTEM_INTERFACE_HPP_
