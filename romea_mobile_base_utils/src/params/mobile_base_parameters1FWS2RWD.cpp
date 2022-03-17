#include "romea_mobile_base_utils/params/mobile_base_parameters1FWS2RWD.hpp"

namespace  {

const std::string front_wheel_steering_joint_param_name="front_left_wheel_steering_joint_name";
const std::string rear_left_wheel_spinning_joint_param_name="rear_left_wheel_spinning_joint_name";
const std::string rear_right_wheel_spinning_joint_param_name="rear_right_wheel_spinning_joint_name";

}
namespace romea {


void declare_mobile_base_info_1FWS2RWD (std::shared_ptr<rclcpp::Node> node,
                                        const std::string & parameters_ns)
{

}

MobileBaseInfo1FWS2RWD get_mobile_base_info_1FWS2RWD (std::shared_ptr<rclcpp::Node> node,
                                                      const std::string & parameters_ns)
{

}

//void declare_joint_mappings_1FWS2RWD(std::shared_ptr<rclcpp::Node> node,
//                                     const std::string & parameters_ns)
//{

//}

//std::map<std::string,std::string> get_joint_mappings_1FWS2RWD(std::shared_ptr<rclcpp::Node> node,
//                                                              const std::string & parameters_ns)
//{

//}

}

