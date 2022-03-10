#ifndef _romea_MobileBaseParameters2TD_hpp_
#define _romea_MobileBaseParameters2TD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2THD.hpp>

namespace romea {

void declare_mobile_base_info_2THD(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & parameters_ns);

MobileBaseInfo2THD get_mobile_base_info_2THD(std::shared_ptr<rclcpp::Node> node,
                                             const std::string & parameters_ns);

void declare_joint_mappings_2THD(std::shared_ptr<rclcpp::Node> node,
                                 const std::string & parameters_ns);

std::map<std::string,std::string> get_joint_mappings_2THD(std::shared_ptr<rclcpp::Node> node,
                                                          const std::string & parameters_ns);

}

#endif