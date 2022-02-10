#ifndef _romea_MobileBaseParameters2FWS2FWD_hpp_
#define _romea_MobileBaseParameters2FWS2FWD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2FWS2FWD.hpp>

namespace romea {


void declare_mobile_base_info(std::shared_ptr<rclcpp::Node> node,
                              const std::string & parameters_ns,
                              const MobileBaseInfo2FWS2FWD & mobile_base_info = MobileBaseInfo2FWS2FWD());

void get_mobile_base_info(std::shared_ptr<rclcpp::Node> node,
                          const std::string & parameters_ns,
                          MobileBaseInfo2FWS2FWD & mobile_base_info);

}

#endif
