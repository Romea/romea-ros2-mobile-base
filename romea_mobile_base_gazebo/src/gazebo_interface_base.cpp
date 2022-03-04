#include "romea_mobile_base_gazebo/gazebo_interface_base.hpp"

namespace romea
{

GazeboInterfaceBase::GazeboInterfaceBase():
  parent_model_()
{
}


//-----------------------------------------------------------------------------
bool GazeboInterfaceBase::initSim(rclcpp::Node::SharedPtr & model_nh,
                                  gazebo::physics::ModelPtr parent_model,
                                  const hardware_interface::HardwareInfo & hardware_info,
                                  sdf::ElementPtr sdf)
{
  nh_=model_nh;
  parent_model_ = parent_model;
  return check_physics_engine_configuration_();

}

//-----------------------------------------------------------------------------
bool GazeboInterfaceBase::check_physics_engine_configuration_()
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
CallbackReturn GazeboInterfaceBase::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
CallbackReturn GazeboInterfaceBase::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
CallbackReturn GazeboInterfaceBase::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
bool GazeboInterfaceBase::init_joint_interfaces_(const hardware_interface::HardwareInfo & hardware_info)
{
  try
  {
    make_joint_interfaces_(hardware_info);
    return true;
  }
  catch (std::runtime_error &e)
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(),e.what());
    return false;
  }
}


}
