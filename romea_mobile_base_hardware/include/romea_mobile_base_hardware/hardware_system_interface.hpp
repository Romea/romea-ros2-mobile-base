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
class HardwareSystemInterface : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  explicit HardwareSystemInterface(
    const std::string & hardware_interface_name = "HardwareInterface");

  virtual ~HardwareSystemInterface() = default;

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

  virtual hardware_interface::return_type load_interface_(
    const hardware_interface::HardwareInfo & hardware_info);

  virtual hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info);

protected:
  std::string hardware_interface_name_;
  std::unique_ptr<HardwareInterface> hardware_interface_;
};

using HardwareSystemInterfac1FAS2FWD = HardwareSystemInterface<HardwareInterface1FAS2FWD>;
using HardwareSystemInterfac1FAS2RWD = HardwareSystemInterface<HardwareInterface1FAS2RWD>;
using HardwareSystemInterfac1FAS4WD = HardwareSystemInterface<HardwareInterface1FAS4WD>;
using HardwareSystemInterface2AS4WD = HardwareSystemInterface<HardwareInterface2AS4WD>;
using HardwareSystemInterface2AS2FWD = HardwareSystemInterface<HardwareInterface2AS2FWD>;
using HardwareSystemInterface2AS2RWD = HardwareSystemInterface<HardwareInterface2AS2RWD>;
using HardwareSystemInterface2FWS2FWD = HardwareSystemInterface<HardwareInterface2FWS2FWD>;
using HardwareSystemInterface2FWS2RWD = HardwareSystemInterface<HardwareInterface2FWS2RWD>;
using HardwareSystemInterface2FWS4WD = HardwareSystemInterface<HardwareInterface2FWS4WD>;
using HardwareSystemInterface2WD = HardwareSystemInterface<HardwareInterface2WD>;
using HardwareSystemInterface4WD = HardwareSystemInterface<HardwareInterface4WD>;
using HardwareSystemInterface4WS4WD = HardwareSystemInterface<HardwareInterface4WS4WD>;
using HardwareSystemInterface2TD = HardwareSystemInterface<HardwareInterface2TD>;
using HardwareSystemInterface2THD = HardwareSystemInterface<HardwareInterface2THD>;
using HardwareSystemInterface2TTD = HardwareSystemInterface<HardwareInterface2TTD>;

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__HARDWARE_SYSTEM_INTERFACE_HPP_
