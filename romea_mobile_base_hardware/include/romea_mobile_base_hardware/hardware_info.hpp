#ifndef _romea_BaseHardwareInterfaceInfo_hpp_
#define _romea_BaseHardwareInterfaceInfo_hpp_

#include <hardware_interface/hardware_info.hpp>
#include <romea_core_common/lexical/LexicalCast.hpp>

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

double get_min(const hardware_interface::InterfaceInfo & interface_info);

double get_max(const hardware_interface::InterfaceInfo & interface_info);


bool has_parameter(const hardware_interface::HardwareInfo & hardware_info,
                   const std::string & parameter_name);

const std::string & get_parameter(const hardware_interface::HardwareInfo & hardware_info,
                                  const std::string & parameter_name);

template <typename T>
T get_parameter(const hardware_interface::HardwareInfo & hardware_info,
                const std::string & parameter_name)
{
  std::string parameter = get_parameter(hardware_info,parameter_name);

  if constexpr(std::is_same_v<T,std::string>)
  {
    return parameter;
  }
  else
  {
    return lexical_cast<T>(parameter);
  }
}

template <typename T>
T get_parameter_or(const hardware_interface::HardwareInfo & hardware_info,
                   const std::string & parameter_name,
                   const T & default_value)
{
  if(has_parameter(hardware_info,parameter_name))
  {
    return get_parameter<T>(hardware_info,parameter_name);
  }
  else
  {
    return default_value;
  }
}

}

#endif
