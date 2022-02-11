#ifndef _romea_MobileBaseParameters2TD_hpp_
#define _romea_MobileBaseParameters2TD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2TD.hpp>

namespace romea {

void declare_mobile_base_info_2TD(std::shared_ptr<rclcpp::Node> node,
                                  const std::string & parameters_ns);

void get_mobile_base_info_2TD(std::shared_ptr<rclcpp::Node> node,
                              const std::string & parameters_ns,
                              MobileBaseInfo2TD & mobile_base_info);

}

#endif
