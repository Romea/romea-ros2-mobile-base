#ifndef _romea_ControllerInterface2TD_hpp_
#define _romea_ControllerInterface2TD_hpp_

#include "spinning_joint_controller_interface.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame2WD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2TD.hpp>

namespace romea
{

class ControllerInterface2TD
{

public:

  using LoanedCommandInterfaces = JointControllerInterface::LoanedCommandInterfaces;
  using LoanedStateInterfaces = JointControllerInterface::LoanedStateInterfaces;

  enum JointIds {
    LEFT_SPROCKET_WHEEL_SPINNING_JOINT_ID,
    RIGHT_SPROCKET_WHEEL_SPINNING_JOINT_ID
  };

public:

  ControllerInterface2TD(const MobileBaseInfo2TD & mobile_base_info);

  void write(const OdometryFrame2WD & command,
             LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(const LoanedStateInterfaces & loaned_state_interfaces,
            OdometryFrame2WD & measurement)const;

public :

  static void declare_joints_names(
      std::shared_ptr<rclcpp::Node> node,
      const std::string & parameters_ns);

  static std::vector<std::string> get_joints_names(
      std::shared_ptr<rclcpp::Node> node,
      const std::string & parameters_ns);

  static std::vector<std::string> hardware_interface_names(
      const std::vector<std::string> & joints_names);

private :

  SpinningJointControllerInterface spinning_joints_;

};


}

#endif
