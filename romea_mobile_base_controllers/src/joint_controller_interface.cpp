#include "romea_mobile_base_controllers/joint_controller_interface.hpp"

namespace  {


}


namespace romea
{

//-----------------------------------------------------------------------------
JointControllerInterface::JointControllerInterface(LoanedCommandInterfaces &loaned_command_interfaces,
                                                   LoanedStateInterfaces &loaned_state_interfaces,
                                                   const std::string &interface_type,
                                                   const std::string &joint_name):
  state_handle_(find_state_handle_(loaned_state_interfaces,interface_type,joint_name)),
  command_handle_(find_command_handle_(loaned_command_interfaces,interface_type,joint_name))
{

}

//-----------------------------------------------------------------------------
std::reference_wrapper<const JointControllerInterface::LoanedStateInterface>
JointControllerInterface::find_state_handle_(LoanedStateInterfaces &loaned_state_interface,
                                             const std::string & interface_type,
                                             const std::string & joint_name)
{

  const auto state_handle = std::find_if(
        loaned_state_interface.cbegin(),
        loaned_state_interface.cend(),
        [&joint_name, &interface_type](const auto & interface)
  {
    return interface.get_name() == joint_name &&
        interface.get_interface_name() == interface_type;
  }
  );

  if (state_handle == loaned_state_interface.cend())
  {
    std::stringstream ss;
    ss << " Unable to obtain joint ";
    ss <<  interface_type ;
    ss << " state handle for ";
    ss <<  joint_name;
    throw(std::runtime_error(ss.str()));
  }

  return std::ref(*state_handle);
}

//-----------------------------------------------------------------------------
std::reference_wrapper<JointControllerInterface::LoanedCommandInterface>
JointControllerInterface::find_command_handle_(LoanedCommandInterfaces & loaned_command_interfaces,
                                               const std::string & interface_type,
                                               const std::string & joint_name)
{


  const auto comman_handle = std::find_if(
        loaned_command_interfaces.begin(),
        loaned_command_interfaces.end(),
        [&joint_name, &interface_type](const auto & interface)
  {
    return interface.get_name() == joint_name &&
        interface.get_interface_name() == interface_type;
  }
  );

  if (comman_handle == loaned_command_interfaces.cend())
  {
    std::stringstream ss;
    ss << " Unable to obtain joint ";
    ss <<  interface_type ;
    ss << " command handle for ";
    ss <<  joint_name;
    throw(std::runtime_error(ss.str()));
  }

  return std::ref(*comman_handle);

}


//-----------------------------------------------------------------------------
const std::string JointControllerInterface::getCommandInterfaceName()const
{
  return command_handle_.get().get_full_name();
}

//-----------------------------------------------------------------------------
const std::string  JointControllerInterface::getStateInterfaceName()const
{
  return state_handle_.get().get_full_name();
}



}
