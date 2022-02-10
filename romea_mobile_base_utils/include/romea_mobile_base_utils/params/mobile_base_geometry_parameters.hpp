#ifndef _romea_MobileBaseGeometryParameters_hpp_
#define _romea_MobileBaseGeometryParameters_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseGeometry.hpp>

namespace romea {

void declare_wheel_info(std::shared_ptr<rclcpp::Node> node,
                        const std::string & parameters_ns);

void declare_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & parameters_ns);

void declare_wheeled_axle_info(std::shared_ptr<rclcpp::Node> node,
                               const std::string & parameters_ns);

void declare_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns);

void declare_two_wheeled_axles_info(std::shared_ptr<rclcpp::Node> node,
                                    const std::string & parameters_ns);



void get_wheel_info(std::shared_ptr<rclcpp::Node> node,
                    const std::string & parameters_ns,
                    Wheel & wheel_info);

void get_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                               const std::string & parameters_ns,
                               ContinuousTrack & track_info);

void get_wheeled_axle_info(std::shared_ptr<rclcpp::Node> node,
                           const std::string & parameters_ns,
                           WheeledAxle & axle_info);

void get_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                      const std::string & parameters_ns,
                                      ContinuousTrackedAxle & axle_info);

void get_two_wheeled_axles_info(std::shared_ptr<rclcpp::Node> node,
                                const std::string & parameters_ns,
                                TwoWheeledAxles & axles_info);



}

#endif
