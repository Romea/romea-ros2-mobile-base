#ifndef _romea_MobileBaseParameters2FWS2FWD_hpp_
#define _romea_MobileBaseParameters2FWS2FWD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2FWS2FWD.hpp>

namespace romea {


void declare_mobile_base_info_2FWS2FWD(std::shared_ptr<rclcpp::Node> node,
                                       const std::string & parameters_ns);

void get_mobile_base_info_2FWS2FWD(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & parameters_ns,
                                   MobileBaseInfo2FWS2FWD & mobile_base_info);

}

#endif
