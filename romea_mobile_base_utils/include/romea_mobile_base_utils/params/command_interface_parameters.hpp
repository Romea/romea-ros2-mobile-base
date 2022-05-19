#ifndef _romea_CommandInterfaceConfiguration_hpp_
#define _romea_CommandInterfaceConfiguration_hpp_

//std
#include <string>

//romea
#include "romea_mobile_base_utils/control/command_interface.hpp"


namespace romea {


void declare_command_interface_configuration(std::shared_ptr<rclcpp::Node> node,
                                             const std::string & parameters_ns);


CommandInterfaceConfiguration get_command_interface_configuration(std::shared_ptr<rclcpp::Node> node,
                                                                  const std::string & parameters_ns);





}// namespace 
#endif
