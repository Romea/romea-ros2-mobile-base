#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include <rclcpp/rclcpp.hpp>

namespace romea {

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
HardwareSystemInterface<HardwareInterface>::HardwareSystemInterface():
  hardware_interface_(nullptr)
{

}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn HardwareSystemInterface<HardwareInterface>::on_init(const hardware_interface::HardwareInfo & hardware_info)
{

  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    hardware_interface_ = std::make_unique<HardwareInterface>(hardware_info,hardware_interface::HW_IF_POSITION);
    return CallbackReturn::SUCCESS;
  }
  catch (std::runtime_error &e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareSystemInterface"),e.what());
    return CallbackReturn::ERROR;
  }

}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
template <typename HardwareInterface>
CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
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
