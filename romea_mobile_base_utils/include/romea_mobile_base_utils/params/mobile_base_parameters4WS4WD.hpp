#ifndef _romea_MobileBaseParameters4WS4WD_hpp_
#define _romea_MobileBaseParameters4WS4WD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo4WS4WD.hpp>

namespace romea {

void declare_mobile_base_info(std::shared_ptr<rclcpp::Node> node,
                              const std::string & parameters_ns,
                              const MobileBaseInfo4WS4WD & mobile_base_info = MobileBaseInfo4WS4WD());

void get_mobile_base_info(std::shared_ptr<rclcpp::Node> node,
                          const std::string & parameters_ns,
                          MobileBaseInfo4WS4WD & mobile_base_info);

}

#endif
