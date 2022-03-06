#ifndef _romea_GazeboInterface2WD_hpp_
#define _romea_GazeboInterface2WD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include <romea_mobile_base_hardware/hardware_interface2WD.hpp>

namespace romea
{

struct GazeboInterface2WD{

  using HardwareInterface = HardwareInterface2WD;

  GazeboInterface2WD(gazebo::physics::ModelPtr parent_model,
                     const hardware_interface::HardwareInfo & hardware_info,
                     const std::string & command_interface_type);

  SpinningJointGazeboInterface left_wheel_spinning_joint;
  SpinningJointGazeboInterface right_wheel_spinning_joint;

};

void write(const HardwareInterface2WD & hardware_interface,
           GazeboInterface2WD & gazebo_interface);

void read(const GazeboInterface2WD & gazebo_interface,
          HardwareInterface2WD & hardware_interface);


}



////romea
//#include "spinning_joint_gazebo_interface.hpp"

////ros
//#include <gazebo_ros2_control/gazebo_system_interface.hpp>


//namespace romea
//{

//class GazeboInterface2WD : public gazebo_ros2_control::GazeboSystemInterface{

//public:

//  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

//public:

//  GazeboInterface2WD();

//  //  void setMeasurement(const OdometryFrame2WD & cmd);

//  //  OdometryFrame2WD getCommand() const;

//  bool initSim(rclcpp::Node::SharedPtr & model_nh,
//               gazebo::physics::ModelPtr parent_model,
//               const hardware_interface::HardwareInfo & hardware_info,
//               sdf::ElementPtr sdf) override;

//  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override ;

//  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

//  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

//  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

//  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//  hardware_interface::return_type read() override;

//  hardware_interface::return_type write() override;

//private :

//  CallbackReturn init_joint_interfaces_(gazebo::physics::ModelPtr parent_model,
//                                        const hardware_interface::HardwareInfo & hardware_info);

//private :


//  std::unique_ptr<SpinningJointGazeboInterface> left_spinning_joint_;
//  std::unique_ptr<SpinningJointGazeboInterface> right_spinning_joint_;
//};


//}

#endif
