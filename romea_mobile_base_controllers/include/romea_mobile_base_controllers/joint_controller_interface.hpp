#ifndef _romea_OdometryJointInterfaces_hpp_
#define _romea_OdometryJointInterfaces_hpp_

#include <controller_interface/controller_interface.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
//#include <hardware_interface/joint_command_interface.h>
//#include <romea_common_utils/params/ros_param.hpp>

namespace romea
{
class JointControllerInterface

{

public:

  using LoanedCommandInterface =  hardware_interface::LoanedCommandInterface;
  using LoanedCommandInterfaces = std::vector<LoanedCommandInterface>;
  using LoanedStateInterface = hardware_interface::LoanedStateInterface;
  using LoanedStateInterfaces = std::vector<LoanedStateInterface>;

public:

  JointControllerInterface(LoanedCommandInterfaces & loaned_command_interfaces,
                           LoanedStateInterfaces & loaned_state_interfaces,
                           const std::string & interface_type,
                           const std::string & joint_name);

  virtual ~JointControllerInterface()=default;

  virtual void setCommand(const double & command)=0;

  virtual double getMeasurement()const=0;

  const std::string getCommandInterfaceName()const;

  const std::string getStateInterfaceName()const;

protected:

  std::reference_wrapper<const LoanedStateInterface>
  find_state_handle_(LoanedStateInterfaces & loaned_state_interfaces,
                     const std::string & interface_type,
                     const std::string & joint_name);

  std::reference_wrapper<LoanedCommandInterface>
  find_command_handle_(LoanedCommandInterfaces &loaned_command_interfaces,
                       const std::string & interface_type,
                       const std::string & joint_name);

protected:

  std::reference_wrapper<const LoanedStateInterface> state_handle_;
  std::reference_wrapper<LoanedCommandInterface> command_handle_  ;
};


}

#endif






