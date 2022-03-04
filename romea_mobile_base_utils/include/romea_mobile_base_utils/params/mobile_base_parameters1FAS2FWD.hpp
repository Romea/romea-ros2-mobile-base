#ifndef _romea_MobileBaseParameters1FAS2FWD_hpp_
#define _romea_MobileBaseParameters1FAS2FWD_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInfo1FAS2FWD.hpp>

namespace romea {

void declare_mobile_base_info_1FAS2FWD(std::shared_ptr<rclcpp::Node> node,
                                       const std::string & parameters_ns);

MobileBaseInfo1FAS2FWD get_mobile_base_info_1FAS2FWD(std::shared_ptr<rclcpp::Node> node,
                                                     const std::string & parameters_ns);


}

#endif