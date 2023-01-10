// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_HARDWARE__STEERING_JOINT_HARDWARE_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__STEERING_JOINT_HARDWARE_INTERFACE_HPP_

// ros
#include <hardware_interface/hardware_info.hpp>

// std
#include <vector>

// local
#include "hardware_handle.hpp"

namespace romea
{

class SteeringJointHardwareInterface
{
public:
  using Command = HardwareCommandInterface;
  using Feedback = HardwareStateInterface;

public:
  explicit SteeringJointHardwareInterface(const hardware_interface::ComponentInfo & joint_info);

  SteeringAngleCommand get_command() const;
  void set_state(const SteeringAngleState & state);

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & hardware_interfaces);
  void export_state_interface(
    std::vector<hardware_interface::StateInterface> & hardware_interfaces);

private:
  Command command_;
  Feedback feedback_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__STEERING_JOINT_HARDWARE_INTERFACE_HPP_
