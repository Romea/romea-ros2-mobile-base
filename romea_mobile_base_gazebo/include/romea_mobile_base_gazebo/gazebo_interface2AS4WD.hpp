#ifndef _romea_BaseHardwareInterface2AS4WD_hpp_
#define _romea_BaseHardwareInterface2AS4WD_hpp_


//romea
#include "steering_joint_hardware_interface.hpp"
#include "spinning_joint_hardware_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame2AS4WD.hpp>
#include <romea_mobile_base_utils/params/hardware_interface_parameters2AS4WD.hpp>

namespace romea
{

class HardwareInterface2AS4WD
{
public:

  using Parameters = HardwareInterfaceConfiguration2AS4WD;

public:

  HardwareInterface2AS4WD(const Parameters & parameters,
                          const std::string & joints_prefix,
                          const hardware_interface::HardwareInfo &hardware_info);

  void setMeasurement(const OdometryFrame2AS4WD & measurement);

  OdometryFrame2AS4WD getCommand() const;

  std::vector<hardware_interface::StateInterface> export_state_interfaces();

  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

private :

  std::unique_ptr<SteeringJointHardwareInterface> front_steering_joint_;
  std::unique_ptr<SteeringJointHardwareInterface> rear_steering_joint_;
  std::unique_ptr<SpinningJointHardwareInterface> front_left_spinning_joint_;
  std::unique_ptr<SpinningJointHardwareInterface> front_right_spinning_joint_;
  std::unique_ptr<SpinningJointHardwareInterface> rear_left_spinning_joint_;
  std::unique_ptr<SpinningJointHardwareInterface> rear_right_spinning_joint_;

};


}

#endif
