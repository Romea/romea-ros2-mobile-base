#ifndef _romea_ControllerInterface2FWS4WD_hpp_
#define _romea_ControllerInterface2FWS4WD_hpp_

#include "controller_interface_common.hpp"
#include <romea_core_mobile_base/odometry/OdometryFrame2FWS4WD.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2FWS4WD.hpp>

namespace romea
{

class ControllerInterface2FWS4WD
{

public:

  using LoanedCommandInterface =  hardware_interface::LoanedCommandInterface;
  using LoanedCommandInterfaces = std::vector<LoanedCommandInterface>;
  using LoanedStateInterface = hardware_interface::LoanedStateInterface;
  using LoanedStateInterfaces = std::vector<LoanedStateInterface>;

  enum JointIds {
    FRONT_LEFT_WHEEL_STEERING_JOINT_ID,
    FRONT_RIGHT_WHEEL_STEERING_JOINT_ID,
    FRONT_LEFT_WHEEL_SPINNING_JOINT_ID,
    FRONT_RIGHT_WHEEL_SPINNING_JOINT_ID,
    REAR_LEFT_WHEEL_SPINNING_JOINT_ID,
    REAR_RIGHT_WHEEL_SPINNING_JOINT_ID,
  };

public:

  ControllerInterface2FWS4WD(const MobileBaseInfo2FWS4WD & mobile_base_info);

  void write(const OdometryFrame2FWS4WD & command,
             LoanedCommandInterfaces & loaned_command_interfaces)const;

  void read(const LoanedStateInterfaces & loaned_state_interfaces,
            OdometryFrame2FWS4WD & measurement)const;

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

  double front_wheels_radius_;
  double rear_wheels_radius_;
};


}

#endif
