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

  JointControllerInterface(const std::string &joint_name,
                           const std::string &interface_type);

  virtual ~JointControllerInterface()=default;

  virtual void set_command(const double & command)=0;

  virtual double get_measurement()const=0;

  const std::string get_command_interface_name()const;

  const std::string get_state_interface_name()const;

  void register_command_interface(LoanedCommandInterface & loaned_command_interface);

  void register_state_interface(LoanedStateInterface & loaned_state_interface);

protected:

  std::string joint_name_;
  std::string interface_type_;

  //TODO use optional_ref
  LoanedStateInterface * state_handle_;
  LoanedCommandInterface * command_handle_  ;

//  std::reference_wrapper<const LoanedStateInterface>
//  find_state_handle_(LoanedStateInterfaces & loaned_state_interfaces,
//                     const std::string & interface_type,
//                     const std::string & joint_name);

//  std::reference_wrapper<LoanedCommandInterface>
//  find_command_handle_(LoanedCommandInterfaces &loaned_command_interfaces,
//                       const std::string & interface_type,
//                       const std::string & joint_name);

//protected:


};


}

#endif






