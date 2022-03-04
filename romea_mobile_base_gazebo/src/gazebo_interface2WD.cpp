#include "romea_mobile_base_gazebo/gazebo_interface2WD.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
GazeboInterface2WD::GazeboInterface2WD(gazebo::physics::ModelPtr parent_model,
                                       const hardware_interface::HardwareInfo & hardware_info,
                                       const std::string & command_interface_type):
  left_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2WD::LEFT_WHEEL_SPINNING_JOINT_ID],command_interface_type),
  right_wheel_spinning_joint(parent_model,hardware_info.joints[HardwareInterface2WD::RIGHT_WHEEL_SPINNING_JOINT_ID],command_interface_type)
{

}

//-----------------------------------------------------------------------------
void write(const HardwareInterface2WD & hardware_interface,
           GazeboInterface2WD & gazebo_interface)
{
  write(hardware_interface.left_wheel_spinning_joint,
        gazebo_interface.left_wheel_spinning_joint);
  write(hardware_interface.right_wheel_spinning_joint,
        gazebo_interface.right_wheel_spinning_joint);
}

//-----------------------------------------------------------------------------
void read(const GazeboInterface2WD & gazebo_interface,
          HardwareInterface2WD & hardware_interface)
{
  read(gazebo_interface.left_wheel_spinning_joint,
       hardware_interface.left_wheel_spinning_joint);
  read(gazebo_interface.right_wheel_spinning_joint,
       hardware_interface.right_wheel_spinning_joint);
}


}


//#include "romea_mobile_base_gazebo/gazebo_interface2WD.hpp"
//#include <romea_mobile_base_utils/params/command_limits_parameters.hpp>

//namespace  {
//const size_t NUMBER_OF_JOINTS =2;
//const size_t LEFT_WHEEL_SPINNING_JOINT_ID=0;
//const size_t RIGHT_WHEEL_SPINNING_JOINT_ID=1;
//}

//namespace romea
//{

////-----------------------------------------------------------------------------
//GazeboInterface2WD::GazeboInterface2WD():
//  left_spinning_joint_(nullptr),
//  right_spinning_joint_(nullptr)
//{

//}

////-----------------------------------------------------------------------------
//bool GazeboInterface2WD::initSim(rclcpp::Node::SharedPtr & model_nh,
//                                 gazebo::physics::ModelPtr parent_model,
//                                 const hardware_interface::HardwareInfo & hardware_info,
//                                 sdf::ElementPtr sdf)
//{
//  nh_ = model_nh;

//  gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

//  std::string physics_type_ = physics->GetType();
//  if (physics_type_.empty()) {
//    RCLCPP_ERROR(nh_->get_logger(), "No physics engine configured in Gazebo.");
//    return false;
//  }

//  return init_joint_interfaces_(parent_model,hardware_info) == CallbackReturn::SUCCESS;
//}

//////-----------------------------------------------------------------------------
////void GazeboInterface2WD::setMeasurement(const OdometryFrame2WD & measurement)
////{
////  left_spinning_joint_->setMeasurement(measurement.leftWheelSpeed);
////  right_spinning_joint_->setMeasurement(measurement.rightWheelSpeed);
////}

//////-----------------------------------------------------------------------------
////OdometryFrame2WD GazeboInterface2WD::getCommand() const
////{
////  OdometryFrame2WD command;
////  command.leftWheelSpeed = left_spinning_joint_->getCommand();
////  command.rightWheelSpeed = right_spinning_joint_->getCommand();
////  return command;
////}


////-----------------------------------------------------------------------------
//CallbackReturn GazeboInterface2WD::on_init(const hardware_interface::HardwareInfo & hardware_info)
//{

//  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
//  {
//    return CallbackReturn::ERROR;
//  }

//  return CallbackReturn::SUCCESS;

//}

////-----------------------------------------------------------------------------
//std::vector<hardware_interface::StateInterface> GazeboInterface2WD::export_state_interfaces()
//{
//  std::vector<hardware_interface::StateInterface> state_interfaces;
//  state_interfaces.emplace_back(left_spinning_joint_->exportStateInterface());
//  state_interfaces.emplace_back(right_spinning_joint_->exportStateInterface());
//  return state_interfaces;
//}

////-----------------------------------------------------------------------------
//std::vector<hardware_interface::CommandInterface> GazeboInterface2WD::export_command_interfaces()
//{
//  std::vector<hardware_interface::CommandInterface> command_interfaces;
//  command_interfaces.emplace_back(left_spinning_joint_->exportCommandInterface());
//  command_interfaces.emplace_back(right_spinning_joint_->exportCommandInterface());
//  return command_interfaces;
//}

////-----------------------------------------------------------------------------
//hardware_interface::return_type GazeboInterface2WD::read()
//{
//  left_spinning_joint_->read();
//  right_spinning_joint_->read();
//  return hardware_interface::return_type::OK;
//}

////-----------------------------------------------------------------------------
//hardware_interface::return_type GazeboInterface2WD::write()
//{
//  left_spinning_joint_->write();
//  right_spinning_joint_->write();
//  return hardware_interface::return_type::OK;
//}


////-----------------------------------------------------------------------------
//CallbackReturn GazeboInterface2WD::init_joint_interfaces_(gazebo::physics::ModelPtr parent_model,
//                                                          const hardware_interface::HardwareInfo & hardware_info)
//{
//  try
//  {
//    left_spinning_joint_ = makeSpinningJointHarwareInterface(
//         parent_model, hardware_info.joints[LEFT_WHEEL_SPINNING_JOINT_ID]);

//    right_spinning_joint_ = makeSpinningJointHarwareInterface(
//         parent_model, hardware_info.joints[RIGHT_WHEEL_SPINNING_JOINT_ID]);
//  }
//  catch (std::runtime_error &e)
//  {
//    RCLCPP_ERROR_STREAM(nh_->get_logger(),e.what());
//    return CallbackReturn::ERROR;
//  }

//  return CallbackReturn::SUCCESS;

//}



