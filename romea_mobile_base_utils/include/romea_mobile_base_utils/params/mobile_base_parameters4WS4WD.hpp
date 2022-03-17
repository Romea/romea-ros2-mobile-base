#ifndef _romea_MobileBaseParameters4WS4WD_hpp_
#define _romea_MobileBaseParameters4WS4WD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo4WS4WD.hpp>

namespace romea {

void declare_mobile_base_info_4WS4WD(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & parameters_ns);

MobileBaseInfo4WS4WD get_mobile_base_info_4WS4WD(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string & parameters_ns);

void declare_joint_mappings_4WS4WD(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & parameters_ns);

std::map<std::string,std::string> get_joint_mappings_4WS4WD(std::shared_ptr<rclcpp::Node> node,
                                                            const std::string & parameters_ns);

}

#endif
