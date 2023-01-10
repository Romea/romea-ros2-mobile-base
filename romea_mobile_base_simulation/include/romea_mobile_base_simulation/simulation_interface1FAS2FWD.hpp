// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE1FAS2FWD_HPP_
#define ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE1FAS2FWD_HPP_

// romea
#include <romea_mobile_base_hardware/hardware_interface1FAS2FWD.hpp>
#include <romea_core_mobile_base/simulation/SimulationControl1FAS2FWD.hpp>

// std
#include <string>
#include <vector>

namespace romea
{

class SimulationInterface1FAS2FWD
{
public:
  SimulationInterface1FAS2FWD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & spinning_joint_command_interface_type);

  SimulationCommand1FAS2FWD get_command()const;
  void set_state(const SimulationState1FAS2FWD & hardware_state);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private:
  HardwareInterface1FAS2FWD hardware_interface_;

  const double wheelbase_;
  const double front_track_;
  const double front_wheel_radius_;
  const double front_hub_carrier_offset_;
  const double rear_track_;
  const double rear_wheel_radius_;
  const double rear_hub_carrier_offset_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_SIMULATION__SIMULATION_INTERFACE1FAS2FWD_HPP_
