#include "romea_mobile_base_hardware/hardware_interface2WD.hpp"

namespace  {
const size_t LEFT_WHEEL_SPINNING_JOINT_ID=0;
const size_t RIGHT_WHEEL_SPINNING_JOINT_ID=1;
}

namespace romea
{

//-----------------------------------------------------------------------------
HardwareInterface2WD::HardwareInterface2WD(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & command_interface_type):
  left_wheel_spinning_joint(hardware_info.joints[LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  right_wheel_spinning_joint(hardware_info.joints[RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{
}


//-----------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
HardwareInterface2WD::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  left_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  right_wheel_spinning_joint.export_state_interfaces(state_interfaces);
  return state_interfaces;
}

//-----------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
HardwareInterface2WD::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  left_wheel_spinning_joint.export_command_interface(command_interfaces);
  right_wheel_spinning_joint.export_command_interface(command_interfaces);
  return command_interfaces;
}


//#include "romea_mobile_base_hardware_interfaces/hardware_interface2WD.hpp"
//#include <romea_mobile_base_utils/params/command_limits_parameters.hpp>

//namespace  {
//const size_t NUMBER_OF_JOINTS =2;
//const size_t LEFT_WHEEL_SPINNING_JOINT_ID=0;
//const size_t RIGHT_WHEEL_SPINNING_JOINT_ID=1;
//}

//namespace romea
//{

////-----------------------------------------------------------------------------
//HardwareInterface2WD::HardwareInterface2WD():
//  left_spinning_joint_(nullptr),
//  right_spinning_joint_(nullptr)
//{

//}

//////-----------------------------------------------------------------------------
////void HardwareInterface2WD::setMeasurement(const OdometryFrame2WD & measurement)
////{
////  left_spinning_joint_->setMeasurement(measurement.leftWheelSpeed);
////  right_spinning_joint_->setMeasurement(measurement.rightWheelSpeed);
////}

//////-----------------------------------------------------------------------------
////OdometryFrame2WD HardwareInterface2WD::getCommand() const
////{
////  OdometryFrame2WD command;
////  command.leftWheelSpeed = left_spinning_joint_->getCommand();
////  command.rightWheelSpeed = right_spinning_joint_->getCommand();
////  return command;
////}


////-----------------------------------------------------------------------------
//CallbackReturn HardwareInterface2WD::on_init(const hardware_interface::HardwareInfo & hardware_info)
//{

//  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
//  {
//    return CallbackReturn::ERROR;
//  }

//  return  init_joint_interfaces_(hardware_info);

//}

////-----------------------------------------------------------------------------
//std::vector<hardware_interface::StateInterface> HardwareInterface2WD::export_state_interfaces()
//{
//  std::vector<hardware_interface::StateInterface> state_interfaces;
//  state_interfaces.emplace_back(left_spinning_joint_->exportStateInterface());
//  state_interfaces.emplace_back(right_spinning_joint_->exportStateInterface());
//  return state_interfaces;
//}

////-----------------------------------------------------------------------------
//std::vector<hardware_interface::CommandInterface> HardwareInterface2WD::export_command_interfaces()
//{
//  std::vector<hardware_interface::CommandInterface> command_interfaces;
//  command_interfaces.emplace_back(left_spinning_joint_->exportCommandInterface());
//  command_interfaces.emplace_back(right_spinning_joint_->exportCommandInterface());
//  return command_interfaces;
//}

////-----------------------------------------------------------------------------
//CallbackReturn HardwareInterface2WD::init_joint_interfaces_(const hardware_interface::HardwareInfo & hardware_info)
//{
//  try
//  {
//    left_spinning_joint_ = makeSpinningJointHarwareInterface(
//          hardware_info.joints[LEFT_WHEEL_SPINNING_JOINT_ID]);

//    right_spinning_joint_ = makeSpinningJointHarwareInterface(
//          hardware_info.joints[RIGHT_WHEEL_SPINNING_JOINT_ID]);

//    return CallbackReturn::SUCCESS;
//  }
//  catch (std::runtime_error &e)
//  {
//    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HardwareInterface2WD"),e.what());
//    return CallbackReturn::ERROR;
//  }
//}

}

