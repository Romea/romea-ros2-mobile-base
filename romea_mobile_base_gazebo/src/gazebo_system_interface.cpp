#include "romea_mobile_base_gazebo/gazebo_system_interface.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template <typename GazeboInterface>
GazeboSystemInterface<GazeboInterface>::GazeboSystemInterface():
  nh_(),
  parent_model_(),
  gazebo_interface_(nullptr),
  hardware_interface_(nullptr)
{
}


//-----------------------------------------------------------------------------
template <typename GazeboInterface>
bool GazeboSystemInterface<GazeboInterface>::initSim(
    rclcpp::Node::SharedPtr & model_nh,
    gazebo::physics::ModelPtr parent_model,
    const hardware_interface::HardwareInfo & hardware_info,
    sdf::ElementPtr sdf)
{
  nh_=model_nh;
  parent_model_ = parent_model;
  return(check_physics_engine_configuration_() &&
        init_gazebo_interfaces_(hardware_info) &&
        init_hardware_interfaces_(hardware_info));

}

//-----------------------------------------------------------------------------
template <typename GazeboInterface>
bool GazeboSystemInterface<GazeboInterface>::check_physics_engine_configuration_()
{
  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

  std::string physics_type_ = physics->GetType();
  if (physics_type_.empty()) {
    RCLCPP_ERROR(this->nh_->get_logger(), "No physics engine configured in Gazebo.");
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
template <typename GazeboInterface>
CallbackReturn GazeboSystemInterface<GazeboInterface>::
on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template <typename GazeboInterface>
CallbackReturn GazeboSystemInterface<GazeboInterface>::
on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template <typename GazeboInterface>
CallbackReturn GazeboSystemInterface<GazeboInterface>::
on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}


//-----------------------------------------------------------------------------
template <typename GazeboInterface>
bool GazeboSystemInterface<GazeboInterface>::
init_gazebo_interfaces_(const hardware_interface::HardwareInfo & hardware_info)
{
  try
  {
    gazebo_interface_ = std::make_unique<GazeboInterface>(parent_model_,hardware_info,"velocity");
    return true;
  }
  catch (std::runtime_error &e)
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(),e.what());
    return false;
  }
}

//-----------------------------------------------------------------------------
template <typename GazeboInterface>
bool GazeboSystemInterface<GazeboInterface>::
init_hardware_interfaces_(const hardware_interface::HardwareInfo & hardware_info)
{
  try
  {
    hardware_interface_ = std::make_unique<HardwareInterface>(hardware_info,"velocity");
    return true;
  }
  catch (std::runtime_error &e)
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(),e.what());
    return false;
  }
}

//-----------------------------------------------------------------------------
template <typename GazeboInterface>
hardware_interface::return_type GazeboSystemInterface<GazeboInterface>::read()
{
  romea::read(*gazebo_interface_,*hardware_interface_);
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template <typename GazeboInterface>
hardware_interface::return_type GazeboSystemInterface<GazeboInterface>::write()
{
  romea::write(*hardware_interface_,*gazebo_interface_);
  return hardware_interface::return_type::OK;
}


template class GazeboSystemInterface<GazeboInterface1FAS2FWD>;
template class GazeboSystemInterface<GazeboInterface1FAS2RWD>;
template class GazeboSystemInterface<GazeboInterface1FWS2RWD>;
template class GazeboSystemInterface<GazeboInterface2FWS2FWD>;
template class GazeboSystemInterface<GazeboInterface2FWS2RWD>;
template class GazeboSystemInterface<GazeboInterface2FWS4WD>;
template class GazeboSystemInterface<GazeboInterface2WD>;
template class GazeboSystemInterface<GazeboInterface4WD>;
template class GazeboSystemInterface<GazeboInterface4WS4WD>;

}