// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// local
#include "romea_mobile_base_controllers/interfaces/controller_interface_common.hpp"

namespace romea
{

std::string hardware_position_interface_name(const std::string joint_name)
{
  return joint_name + "/" + hardware_interface::HW_IF_POSITION;
}

std::string hardware_velocity_interface_name(const std::string joint_name)
{
  return joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
}

std::string hardware_effort_interface_name(const std::string joint_name)
{
  return joint_name + "/" + hardware_interface::HW_IF_EFFORT;
}

}  // namespace romea
