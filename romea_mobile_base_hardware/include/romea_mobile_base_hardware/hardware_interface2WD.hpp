#ifndef _romea_HardwareInterface2WD_hpp_
#define _romea_HardwareInterface2WD_hpp_

#include "hardware_spinning_joint_interface.hpp"

namespace romea
{

struct HardwareInterface2WD{

  HardwareInterface2WD(const hardware_interface::HardwareInfo & hardware_info,
                       const std::string & command_interface_type);

  HardwareSpinningJointInterface left_wheel_spinning_joint;
  HardwareSpinningJointInterface right_wheel_spinning_joint;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

};


}

#endif


//#ifndef _romea_BaseHardwareInterface2WD_hpp_
//#define _romea_BaseHardwareInterface2WD_hpp_

////romea
//#include "spinning_joint_hardware_interface.hpp"

////ros
//#include <hardware_interface/system_interface.hpp>


//namespace romea
//{

//struct HardwareJointInterfaces2WD
//{
//  enum ROS2_CONTROL_JOINT_IDS
//  {
//    LEFT_SPINNING_JOINT=0;
//    RIGHT_SPINNING_JOINT=1;
//  }

//  HardwareJointInterfaces2WD(const hardware_interface::HardwareInfo & hardware_info);

//  SpinningJointHardwareInterface left_spinning_joint_velocity_interface;
//  SpinningJointHardwareInterface right_spinning_joint_velocity_interface;

//  std::vector<hardware_interface::StateInterface> export_state_interfaces();
//  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

//};

//class HardwareSystemInterface2WD : public hardware_interface::SystemInterface{


//public:

//  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

//public:

//  HardwareInterface2WD();

//  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override ;

//  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

//  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

//  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

//  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//private :

//  CallbackReturn init_joint_interfaces_(const hardware_interface::HardwareInfo & hardware_info);

//private :

//  std::unique_ptr<SpinningJointHardwareInterface> left_spinning_joint_;
//  std::unique_ptr<SpinningJointHardwareInterface> right_spinning_joint_;

//};


//}
