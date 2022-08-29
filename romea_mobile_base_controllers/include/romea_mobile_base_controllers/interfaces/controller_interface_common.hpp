#ifndef _romea_ControllerInterfaceCommon_hpp_
#define _romea_ControllerInterfaceCommon_hpp_

#include <controller_interface/controller_interface.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace romea
{

  std::string hardware_position_interface_name(const std::string joint_name);

  std::string hardware_velocity_interface_name(const std::string joint_name);

  std::string hardware_effort_interface_name(const std::string joint_name);

}

#endif






