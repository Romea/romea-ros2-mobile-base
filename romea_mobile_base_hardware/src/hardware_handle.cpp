#include "romea_mobile_base_hardware/hardware_handle.hpp"
#include "romea_mobile_base_hardware/hardware_info.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
HardwareCommandInterface::HardwareCommandInterface(const hardware_interface::ComponentInfo &joint_info,
                                                   const std::string & interface_type):
  command_(std::numeric_limits<double>::quiet_NaN()),
  command_min_(-std::numeric_limits<double>::max()),
  command_max_( std::numeric_limits<double>::max()),
  joint_name_(joint_info.name),
  interface_type_(interface_type)
{
  //TODO initialize
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
  state_(std::numeric_limits<double>::quiet_NaN()),
  state_min_(-std::numeric_limits<double>::max()),
  state_max_( std::numeric_limits<double>::max()),
  joint_name_(joint_info.name),
  interface_type_(interface_type)
{
  //TODO initialize
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
