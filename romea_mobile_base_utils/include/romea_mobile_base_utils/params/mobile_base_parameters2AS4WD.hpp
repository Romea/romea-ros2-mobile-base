#ifndef _romea_MobileBaseParameters2AS4WD_hpp_
#define _romea_MobileBaseParameters2AS4WD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo2AS4WD.hpp>

namespace romea {


void declare_mobile_base_info_2AS4WD(std::shared_ptr<rclcpp::Node> node,
                                     const std::string & parameters_ns);

void get_mobile_base_info_2AS4WD(std::shared_ptr<rclcpp::Node> node,
                                 const std::string & parameters_ns,
                                 MobileBaseInfo2AS4WD & mobile_base_info);


}

#endif
