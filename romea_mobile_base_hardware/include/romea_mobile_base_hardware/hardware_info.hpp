#ifndef _romea_BaseHardwareInterfaceInfo_hpp_
#define _romea_BaseHardwareInterfaceInfo_hpp_

#include <hardware_interface/hardware_info.hpp>

namespace romea
{


const  hardware_interface::ComponentInfo &
get_joint_info(const hardware_interface::HardwareInfo & hardware_info,
               const std::string & joint_name);

const hardware_interface::InterfaceInfo &
get_command_interface_info(const hardware_interface::ComponentInfo & component_info,
                           const std::string & interface_name);

const hardware_interface::InterfaceInfo &
get_state_interface_info(const hardware_interface::ComponentInfo &component_info,
                         const std::string & interface_name);

const std::string & get_parameter(const hardware_interface::HardwareInfo & hardware_info,
                                  const std::string & parameter_name);

}

#endif
