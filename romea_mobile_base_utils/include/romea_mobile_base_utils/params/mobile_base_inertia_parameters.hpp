#ifndef _romea_MobileBaseInertiaParameters_hpp_
#define _romea_MobileBaseInertiaParameters_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseInertia.hpp>

namespace romea {

void declare_inertia_info(std::shared_ptr<rclcpp::Node> node,
                          const std::string & parameters_ns);

void get_inertia_info(std::shared_ptr<rclcpp::Node> node,
                      const std::string & parameters_ns,
                      MobileBaseInertia & inertia_info);

MobileBaseInertia get_inertia_info(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & parameters_ns);

}

#endif
