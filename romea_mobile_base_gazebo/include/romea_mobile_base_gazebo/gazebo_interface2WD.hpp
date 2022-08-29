#ifndef _romea_GazeboInterface2WD_hpp_
#define _romea_GazeboInterface2WD_hpp_

#include "spinning_joint_gazebo_interface.hpp"
#include <romea_core_mobile_base/simulation/SimulationControl2WD.hpp>

namespace romea
{

class GazeboInterface2WD{


public :

  GazeboInterface2WD(gazebo::physics::ModelPtr parent_model,
                     const hardware_interface::HardwareInfo & hardware_info,
                     const std::string & command_interface_type);


  SimulationState2WD get_state() const;
  void set_command(const SimulationCommand2WD &command);

private :

  SpinningJointGazeboInterface left_wheel_spinning_joint_;
  SpinningJointGazeboInterface right_wheel_spinning_joint_;

};

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
