#ifndef _romea_GazeboInterface2WD_hpp_
#define _romea_GazeboInterface2WD_hpp_

//romea
#include "joint_gazebo_interface.hpp"

//ros
#include <gazebo_ros2_control/gazebo_system_interface.hpp>


namespace romea
{

class GazeboInterfaceBase : public gazebo_ros2_control::GazeboSystemInterface{

public:

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:

  GazeboInterfaceBase();

  bool initSim(rclcpp::Node::SharedPtr & model_nh,
               gazebo::physics::ModelPtr parent_model,
               const hardware_interface::HardwareInfo & hardware_info,
               sdf::ElementPtr sdf) override;

  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override ;

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private :

  bool check_physics_engine_configuration_();

  bool init_joint_interfaces_(const hardware_interface::HardwareInfo & hardware_info);

  virtual void make_joint_interfaces_(const hardware_interface::HardwareInfo & hardware_info)=0;

private :

//  std::vector<std::unique_ptr<JointInterfaceInfo>> joints_;
  gazebo::physics::ModelPtr parent_model_;

};


}

#endif
