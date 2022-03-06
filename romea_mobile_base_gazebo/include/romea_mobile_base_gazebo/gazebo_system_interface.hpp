#ifndef _romea_GazeboSystemInterface_hpp_
#define _romea_GazeboSystemInterface_hpp_

//romea
#include "gazebo_interface1FAS2FWD.hpp"
#include "gazebo_interface1FAS2RWD.hpp"
#include "gazebo_interface1FWS2RWD.hpp"
#include "gazebo_interface2FWS2FWD.hpp"
#include "gazebo_interface2FWS2RWD.hpp"
#include "gazebo_interface2FWS4WD.hpp"
#include "gazebo_interface2WD.hpp"
#include "gazebo_interface4WD.hpp"
#include "gazebo_interface4WS4WD.hpp"

//ros
#include <gazebo_ros2_control/gazebo_system_interface.hpp>


namespace romea
{

template <typename GazeboInterface>
class GazeboSystemInterface : public gazebo_ros2_control::GazeboSystemInterface{

public:

  using HardwareInterface = typename GazeboInterface::HardwareInterface;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:

  GazeboSystemInterface();

  bool initSim(rclcpp::Node::SharedPtr & model_nh,
               gazebo::physics::ModelPtr parent_model,
               const hardware_interface::HardwareInfo & hardware_info,
               sdf::ElementPtr sdf) override;

  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override ;

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  virtual hardware_interface::return_type read();

  virtual hardware_interface::return_type write();


private :

  bool check_physics_engine_configuration_();

  bool init_gazebo_interfaces_(const hardware_interface::HardwareInfo & hardware_info);

  bool init_hardware_interfaces_(const hardware_interface::HardwareInfo & hardware_info);

private :

  rclcpp::Node::SharedPtr nh_;
  gazebo::physics::ModelPtr parent_model_;
  std::unique_ptr<GazeboInterface> gazebo_interface_;
  std::unique_ptr<HardwareInterface> hardware_interface_;


};

using GazeboInterfaceBase1FAS2FWD = GazeboSystemInterface<GazeboInterface1FAS2FWD>;
using GazeboInterfaceBase1FAS2RWD = GazeboSystemInterface<GazeboInterface1FAS2RWD>;
using GazeboInterfaceBase1FWS2FWD = GazeboSystemInterface<GazeboInterface1FWS2RWD>;
using GazeboInterfaceBase2FWS2FWD = GazeboSystemInterface<GazeboInterface2FWS2FWD>;
using GazeboInterfaceBase2FWS2RWD = GazeboSystemInterface<GazeboInterface2FWS2RWD>;
using GazeboInterfaceBase2FWS4WD = GazeboSystemInterface<GazeboInterface2FWS4WD>;
using GazeboInterfaceBase2WD = GazeboSystemInterface<GazeboInterface2WD>;
using GazeboInterfaceBase4WD = GazeboSystemInterface<GazeboInterface4WD>;
using GazeboInterfaceBase4WS4WD = GazeboSystemInterface<GazeboInterface4WS4WD>;

}

#endif
