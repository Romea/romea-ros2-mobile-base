#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace romea {

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
HardwareSystemInterface<HardwareInterface>::HardwareSystemInterface():
  hardware_interface_(nullptr)
{
  RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"HardwareSystemInterface");
  std::cout << " HardwareSystemInterface "<< std::endl;
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn HardwareSystemInterface<HardwareInterface>::on_init(const hardware_interface::HardwareInfo & hardware_info)
{

  //  RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"on_init");
  //  RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"),hardware_info.name);
  //  for(const auto & joint : hardware_info.joints)
  //  {
  //    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"),joint.name<<" " <<joint.type);

  //    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"state interfaces");
  //    for(const auto & state_interface :joint.state_interfaces)
  //    {
  //      RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface")," "<<state_interface.name);
  //    }
  //    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"command interfaces");
  //    for(const auto & command_interface :joint.command_interfaces)
  //    {
  //      RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface")," "<<command_interface.name);
  //    }

  //  }


  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  if(load_info_(hardware_info)==hardware_interface::return_type::OK &&
     load_interface_(hardware_info)==hardware_interface::return_type::OK)
  {
    return CallbackReturn::SUCCESS;
  }
  else
  {
    return CallbackReturn::ERROR;
  }

}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
hardware_interface::return_type HardwareSystemInterface<HardwareInterface>::
load_info_(const hardware_interface::HardwareInfo & /*hardware_info*/)
{
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
hardware_interface::return_type HardwareSystemInterface<HardwareInterface>::
load_interface_(const hardware_interface::HardwareInfo &hardware_info)
{
  try
  {
    hardware_interface_ = std::make_unique<HardwareInterface>(hardware_info,hardware_interface::HW_IF_VELOCITY);
    return hardware_interface::return_type::OK;
  }
  catch (std::runtime_error &e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"),e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn HardwareSystemInterface<HardwareInterface>::on_configure(const rclcpp_lifecycle::State & previous_state )
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"on_configure : previous state " <<int(previous_state.id())<<" "<<previous_state.label());
  if(connect_()==hardware_interface::return_type::OK)
  {
    return CallbackReturn::SUCCESS;
  }
  else
  {
    return CallbackReturn::FAILURE;
  }
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn HardwareSystemInterface<HardwareInterface>::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"on_cleanup : previous state " <<int(previous_state.id())<<" "<<previous_state.label());
  if(disconnect_()==hardware_interface::return_type::OK)
  {
    return CallbackReturn::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"on_cleanup raised error");
    return CallbackReturn::ERROR;
  }
}


//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn HardwareSystemInterface<HardwareInterface>::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"on_activate : previous state " <<int(previous_state.id())<<" "<<previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn HardwareSystemInterface<HardwareInterface>::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  //send null command
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"on_deactivate : previous state" <<int(previous_state.id())<<" "<<previous_state.label());
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn HardwareSystemInterface<HardwareInterface>::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"on_shutdownn : previous state " <<int(previous_state.id())<<" "<<previous_state.label());

  if(int(previous_state.id()) == 1)
  {
    return CallbackReturn::SUCCESS;
  }
  else if(disconnect_()==hardware_interface::return_type::OK)
  {
    return CallbackReturn::SUCCESS;
  }
  else
  {
    return CallbackReturn::ERROR;
  }
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn HardwareSystemInterface<HardwareInterface>::on_error(const rclcpp_lifecycle::State &  previous_state)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("HardwareSystemInterface"),"on_error : previous state " <<int(previous_state.id())<<" "<<previous_state.label());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
std::vector<hardware_interface::StateInterface> HardwareSystemInterface<HardwareInterface>::export_state_interfaces()
{
  return hardware_interface_->export_state_interfaces();
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
std::vector<hardware_interface::CommandInterface> HardwareSystemInterface<HardwareInterface>::export_command_interfaces()
{
  return hardware_interface_->export_command_interfaces();
}

template class HardwareSystemInterface<HardwareInterface2WD>;
template class HardwareSystemInterface<HardwareInterface4WD>;
template class HardwareSystemInterface<HardwareInterface4WS4WD>;
template class HardwareSystemInterface<HardwareInterface2FWS4WD>;
template class HardwareSystemInterface<HardwareInterface2FWS2RWD>;
template class HardwareSystemInterface<HardwareInterface2FWS2FWD>;
template class HardwareSystemInterface<HardwareInterface2AS4WD>;
template class HardwareSystemInterface<HardwareInterface1FAS2FWD>;
template class HardwareSystemInterface<HardwareInterface1FAS2RWD>;
template class HardwareSystemInterface<HardwareInterface1FWS2RWD>;

}
