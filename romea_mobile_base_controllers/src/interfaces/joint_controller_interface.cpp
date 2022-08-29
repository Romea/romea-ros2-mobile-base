//#include "romea_mobile_base_controllers/interfaces/joint_controller_interface.hpp"
//#include <sstream>

//namespace romea
//{

////-----------------------------------------------------------------------------
//JointControllerInterface::JointControllerInterface(const std::string &joint_name,
//                                                   const std::string &interface_type):
//  joint_name_(joint_name),
//  interface_type_(interface_type)
////  state_handle_(nullptr),
////  command_handle_(nullptr)
//{
//}

//////-----------------------------------------------------------------------------
////void JointControllerInterface::register_command_interface(LoanedCommandInterface & loaned_command_interface)
////{
////  if(loaned_command_interface.get_name()==joint_name_ &&
////     loaned_command_interface.get_interface_name() == interface_type_)
////  {
////    command_handle_ = & loaned_command_interface;
////  }
////  else
////  {
////    std::stringstream ss;
////    ss << " Joint interface ";
////    ss << get_command_interface_name();
////    ss << " : unable to register command interface ";
////    ss << loaned_command_interface.get_full_name();
////    throw std::runtime_error(ss.str());
////  }
////}

//////-----------------------------------------------------------------------------
////void JointControllerInterface::register_state_interface(LoanedStateInterface & loaned_state_interface)
////{
////  if(loaned_state_interface.get_name()==joint_name_ &&
////     loaned_state_interface.get_interface_name() == interface_type_)
////  {
////    state_handle_ = & loaned_state_interface;
////  }
////  else
////  {
////    std::stringstream ss;
////    ss << " Joint interface ";
////    ss << get_command_interface_name();
////    ss << " : unable to register state interface ";
////    ss << loaned_state_interface.get_full_name();
////    throw std::runtime_error(ss.str());
////  }
////}

//////-----------------------------------------------------------------------------
////const std::string JointControllerInterface::get_command_interface_name()const
////{
////  return joint_name_+"/"+interface_type_;
////  //  return command_handle_.get().get_full_name();
////}

//////-----------------------------------------------------------------------------
////const std::string  JointControllerInterface::get_state_interface_name()const
////{
////  return joint_name_+"/"+interface_type_;
////  //  return state_handle_.get().get_full_name();
////}



//}
