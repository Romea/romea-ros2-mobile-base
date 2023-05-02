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


#ifndef ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2TTD_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2TTD_HPP_

// std
#include <string>
#include <vector>

// romea
#include "romea_mobile_base_hardware/hardware_interface2TTD.hpp"
#include "romea_core_mobile_base/simulation/SimulationControl2TTD.hpp"


namespace romea
{

class SimulationInterface2TTD
{
public:
  SimulationInterface2TTD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type);

  SimulationCommand2TTD get_command()const;
  void set_state(const SimulationState2TTD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  HardwareInterface2TTD hardware_interface_;

  const double idler_wheel_radius_;
  const double roller_wheel_radius_;
  const double sprocket_wheel_radius_;
  const double track_thickness_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE2TTD_HPP_
