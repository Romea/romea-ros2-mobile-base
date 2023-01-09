// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE_COMMON_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE_COMMON_HPP_

// ros
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// std
#include <string>


namespace romea
{

std::string hardware_position_interface_name(const std::string joint_name);

std::string hardware_velocity_interface_name(const std::string joint_name);

std::string hardware_effort_interface_name(const std::string joint_name);

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__INTERFACES__CONTROLLER_INTERFACE_COMMON_HPP_
