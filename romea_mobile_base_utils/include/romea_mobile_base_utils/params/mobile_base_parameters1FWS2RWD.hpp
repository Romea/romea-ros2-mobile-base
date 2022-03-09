#ifndef _romea_MobileBaseParameters1FWS2RWD_hpp_
#define _romea_MobileBaseParameters1FWS2RWD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo1FWS2RWD.hpp>

namespace romea {

void declare_mobile_base_info_1FWS2RWD(std::shared_ptr<rclcpp::Node> node,
                                       const std::string & parameters_ns);


MobileBaseInfo1FWS2RWD  get_mobile_base_info_1FWS2RWD(std::shared_ptr<rclcpp::Node> node,
                                                      const std::string & parameters_ns);

void declare_joint_mappings_1FWS2RWD(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & parameters_ns);

std::map<std::string,std::string> get_joint_mappings_1FWS2RWD(std::shared_ptr<rclcpp::Node> node,
                                                              const std::string & parameters_ns);

}

#endif
