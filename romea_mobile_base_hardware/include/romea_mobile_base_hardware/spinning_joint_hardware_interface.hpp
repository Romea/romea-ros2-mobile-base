// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_HARDWARE__SPINNING_JOINT_HARDWARE_INTERFACE_HPP_
#define ROMEA_MOBILE_BASE_HARDWARE__SPINNING_JOINT_HARDWARE_INTERFACE_HPP_


// ros
#include <hardware_interface/hardware_info.hpp>

// std
#include <string>
#include <vector>

// local
#include "romea_mobile_base_hardware/hardware_handle.hpp"

namespace romea
{

RotationalMotionControlType toRotationalMotionCommandType(const std::string & interface_type);


class SpinningJointHardwareInterface
{
public:
  using Command = HardwareCommandInterface;
  using CommandType = RotationalMotionControlType;

  struct Feedback
  {
    explicit Feedback(const hardware_interface::ComponentInfo & joint_info);
    HardwareStateInterface position;
    HardwareStateInterface velocity;
    HardwareStateInterface torque;

    void set_state(const RotationalMotionState & state);

    void export_state_interfaces(
      std::vector<hardware_interface::StateInterface> & state_interfaces);
  };

public:
  SpinningJointHardwareInterface(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & spinning_joint_command_interface_type);

  double get_command() const;
  void set_state(const RotationalMotionState & state);

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces);
  void export_state_interfaces(std::vector<hardware_interface::StateInterface> & state_interfaces);

private:
  Command command_;
  Feedback feedback_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_HARDWARE__SPINNING_JOINT_HARDWARE_INTERFACE_HPP_
