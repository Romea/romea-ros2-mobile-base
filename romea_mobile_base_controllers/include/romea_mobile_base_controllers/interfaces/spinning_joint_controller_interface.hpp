//#ifndef _romea_SpinningJointInterface_hpp_
//#define _romea_SpinningJointInterface_hpp_

//#include "joint_controller_interface.hpp"

//namespace romea
//{


//class SpinningJointControllerInterface
//{
//public:

//  using LoanedStateInterface = hardware_interface::LoanedStateInterface;
//  using LoanedCommandInterface = hardware_interface::LoanedCommandInterface;

//public:

//  SpinningJointControllerInterface(const double & wheel_radius);

//  void write(const double & command, LoanedCommandInterface & loaned_command_interface)const;

//  void read(const LoanedStateInterface & loaned_state_interface, double & measurement) const;

//  static std::string hardware_interface_name(const std::string & joint_name);

//private:

//  double wheel_radius_;
//};

//}
//#endif
