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
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ros
#include "hardware_interface/system_interface.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_interface_base.hpp"
#include "romea_mobile_base_hardware/hardware_system_interface_legacy.hpp"

namespace romea
{
namespace ros2
{

class HardwareSystemInterface : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  explicit HardwareSystemInterface(
    const std::string & hardware_interface_name = "HardwareSystemInterface");

  ~HardwareSystemInterface() override = default;

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
  virtual hardware_interface::return_type connect_() = 0;

  virtual hardware_interface::return_type disconnect_() = 0;

  virtual hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info);

  virtual hardware_interface::return_type load_interfaces_(
    const hardware_interface::HardwareInfo & hardware_info);

  HardwareInterfaceBase & hardware_interface_(const std::string & interface_name);

  const HardwareInterfaceBase & hardware_interface_(const std::string & interface_name) const;

  template<typename HardwareInterface>
  HardwareInterface & hardware_interface(const std::string & interface_name)
  {
    return dynamic_cast<HardwareInterface &>(hardware_interface_(interface_name));
  }

  template<typename HardwareInterface>
  const HardwareInterface & hardware_interface(const std::string & interface_name) const
  {
    return dynamic_cast<const HardwareInterface &>(hardware_interface_(interface_name));
  }

protected:
  std::string hardware_interface_name_;
  std::vector<std::string> hardware_interface_names_;
  std::unordered_map<std::string, std::unique_ptr<HardwareInterfaceBase>> hardware_interfaces_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_SYSTEM_INTERFACE_HPP_
