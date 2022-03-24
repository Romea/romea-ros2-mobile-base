#include "romea_mobile_base_hardware/hardware_handle.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareCommandInterface::HardwareCommandInterface(const hardware_interface::InterfaceInfo & interface_info,
                                                   const std::string & joint_name):
  command_(0.0),
  command_min_(romea::get_min(interface_info)),
  command_max_(romea::get_max(interface_info)),
  joint_name_(joint_name),
  interface_type_(interface_info.name)
{

}

//-----------------------------------------------------------------------------
HardwareCommandInterface::HardwareCommandInterface(const hardware_interface::ComponentInfo &joint_info,
                                                   const std::string & interface_type):
  HardwareCommandInterface(get_command_interface_info(joint_info,interface_type),joint_info.name)
{
}

//-----------------------------------------------------------------------------
double HardwareCommandInterface::get() const
{
  return command_;
}

//-----------------------------------------------------------------------------
void HardwareCommandInterface::export_interface(std::vector<hardware_interface::CommandInterface> & hardware_interfaces)
{
  using hardware_interface::CommandInterface;
  hardware_interfaces.push_back(CommandInterface(joint_name_,interface_type_,&command_));
}


//-----------------------------------------------------------------------------
const std::string & HardwareCommandInterface::get_interface_type() const
{
  return interface_type_;
}

//-----------------------------------------------------------------------------
const std::string & HardwareCommandInterface::get_joint_name() const
{
  return joint_name_;
}

//-----------------------------------------------------------------------------
HardwareStateInterface::HardwareStateInterface(const hardware_interface::ComponentInfo & joint_info,
                                               const std::string & interface_type):
  HardwareStateInterface(get_state_interface_info(joint_info,interface_type),joint_info.name)
{
}

//-----------------------------------------------------------------------------
HardwareStateInterface::HardwareStateInterface(const hardware_interface::InterfaceInfo & interface_info,
                                               const std::string & joint_name):
  state_(0.0),
  state_min_(romea::get_min(interface_info)),
  state_max_(romea::get_max(interface_info)),
  joint_name_(joint_name),
  interface_type_(interface_info.name)

{

}


//-----------------------------------------------------------------------------
void HardwareStateInterface::set(const double & state)
{
  state_=state;
}

//-----------------------------------------------------------------------------
void HardwareStateInterface::export_interface(std::vector<hardware_interface::StateInterface> & state_interfaces)
{
  using hardware_interface::StateInterface;
  state_interfaces.push_back(StateInterface(joint_name_,interface_type_,&state_));
}

//-----------------------------------------------------------------------------
const std::string & HardwareStateInterface::get_interface_type() const
{
  return interface_type_;
}

//-----------------------------------------------------------------------------
const std::string & HardwareStateInterface::get_joint_name() const
{
  return joint_name_;
}


}
