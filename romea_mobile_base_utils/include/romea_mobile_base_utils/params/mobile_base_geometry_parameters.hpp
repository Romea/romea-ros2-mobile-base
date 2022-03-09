#ifndef _romea_MobileBaseGeometryParameters_hpp_
#define _romea_MobileBaseGeometryParameters_hpp_

#include <rclcpp/node.hpp>
#include <romea_core_mobile_base/info/MobileBaseGeometry.hpp>

namespace romea {

void declare_wheel_info(std::shared_ptr<rclcpp::Node> node,
                        const std::string & parameters_ns);

void declare_track_wheel_info(std::shared_ptr<rclcpp::Node> node,
                              const std::string & parameters_ns);

void declare_flat_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                                        const std::string & parameters_ns);

void declare_triangular_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                                              const std::string & parameters_ns);

void declare_wheeled_axle_info(std::shared_ptr<rclcpp::Node> node,
                               const std::string & parameters_ns);

void declare_flat_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                               const std::string & parameters_ns);

void declare_triangular_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                                     const std::string & parameters_ns);

void declare_two_wheeled_axles_info(std::shared_ptr<rclcpp::Node> node,
                                    const std::string & parameters_ns);


Wheel get_wheel_info(std::shared_ptr<rclcpp::Node> node,
                     const std::string & parameters_ns);

TrackWheel get_track_wheel_info(std::shared_ptr<rclcpp::Node> node,
                                const std::string & parameters_ns);

FlatContinuousTrack get_flat_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                                                   const std::string & parameters_ns);

TriangularContinuousTrack get_triangular_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                                                               const std::string & parameters_ns);

WheeledAxle get_wheeled_axle_info(std::shared_ptr<rclcpp::Node> node,
                                  const std::string & parameters_ns);

FlatContinuousTrackedAxle get_flat_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                                                const std::string & parameters_ns);

TriangularContinuousTrackedAxle get_triangular_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                                                            const std::string & parameters_ns);

TwoWheeledAxles get_two_wheeled_axles_info(std::shared_ptr<rclcpp::Node> node,
                                           const std::string & parameters_ns);



}

#endif
