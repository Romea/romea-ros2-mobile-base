#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"
#include <romea_common_utils/params/node_parameters.hpp>
#include <limits>

namespace
{
const std::string radius_param_name="radius";
const std::string width_param_name="width";
const std::string hub_carrier_offset_param_name="hub_carrier_offset";

const std::string x_param_name = "x";
const std::string z_param_name = "z";
const std::string thickness_param_name = "thickness";
const std::string sprocket_wheel_param_name = "sprocket_wheel";
const std::string idler_wheel_param_name = "idler_wheel";
const std::string front_idler_wheel_param_name = "front_idler_wheel";
const std::string rear_idler_wheel_param_name = "rear_idler_wheel";
const std::string rollers_param_name = "rollers";


const std::string wheels_param_name="wheels";
const std::string wheels_distance_param_name="wheels_distance";

const std::string tracks_param_name="tracks";
const std::string tracks_distance_param_name="tracks_distance";


const std::string front_axle_param_name="front_axle";
const std::string rear_axle_param_name="rear_axle";
const std::string axle_distance_param_name="axles_distance";

void declare_continuous_track_common_info(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  romea::declare_parameter<double>(node,parameters_ns,width_param_name);
  romea::declare_parameter<double>(node,parameters_ns,thickness_param_name);
  romea::declare_track_wheels_info(node,romea::full_param_name(parameters_ns,rollers_param_name));
}

romea::ContinuousTrackBase get_continuous_track_common_info(std::shared_ptr<rclcpp::Node> node,
                                                            const std::string & parameters_ns)
{
  return {romea::get_parameter<double>(node,parameters_ns,width_param_name),
        romea::get_parameter<double>(node,parameters_ns,thickness_param_name),
        romea::get_track_wheels_info(node,romea::full_param_name(parameters_ns,rollers_param_name))};
}

}


namespace romea {

//-----------------------------------------------------------------------------
void declare_wheel_info(std::shared_ptr<rclcpp::Node> node,
                        const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,radius_param_name);
  declare_parameter<double>(node,parameters_ns,width_param_name);
  declare_parameter_with_default<double>(
        node,parameters_ns,hub_carrier_offset_param_name,0.0);
}

//-----------------------------------------------------------------------------
Wheel get_wheel_info(std::shared_ptr<rclcpp::Node> node,
                     const std::string & parameters_ns)
{
  return {get_parameter<double>(node,parameters_ns,radius_param_name),
        get_parameter<double>(node,parameters_ns,width_param_name),
        get_parameter<double>(node,parameters_ns,hub_carrier_offset_param_name)};

}


//-----------------------------------------------------------------------------
void declare_track_wheel_info(std::shared_ptr<rclcpp::Node> node,
                              const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,radius_param_name);
  declare_parameter<double>(node,parameters_ns,x_param_name);
  declare_parameter_with_default<double>(node,parameters_ns,z_param_name,std::numeric_limits<double>::quiet_NaN());
}

//-----------------------------------------------------------------------------
TrackWheel get_track_wheel_info(std::shared_ptr<rclcpp::Node> node,
                                const std::string & parameters_ns)
{
  auto radius = get_parameter<double>(node,parameters_ns,radius_param_name);
  auto x = get_parameter<double>(node,parameters_ns,x_param_name);
  auto z = get_parameter<double>(node,parameters_ns,z_param_name);
  return {radius,x,std::isfinite(z) ? z : radius};
}


//-----------------------------------------------------------------------------
void declare_track_wheels_info(std::shared_ptr<rclcpp::Node> node,
                               const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,radius_param_name);
  declare_vector_parameter<double>(node,parameters_ns,x_param_name);
  declare_parameter_with_default<double>(node,parameters_ns,z_param_name,std::numeric_limits<double>::quiet_NaN());
}

//-----------------------------------------------------------------------------
TrackWheels get_track_wheels_info(std::shared_ptr<rclcpp::Node> node,
                                  const std::string & parameters_ns)
{
  auto radius = get_parameter<double>(node,parameters_ns,radius_param_name);
  auto x = get_vector_parameter<double>(node,parameters_ns,x_param_name);
  auto z = get_parameter<double>(node,parameters_ns,z_param_name);
  return {radius,x,std::isfinite(z) ? z : radius};
}


//-----------------------------------------------------------------------------
template<>
void declare_continuous_track_info<ContinuousTrack>(std::shared_ptr<rclcpp::Node> node,
                                                    const std::string & parameters_ns)
{
  declare_continuous_track_common_info(node,parameters_ns);
  declare_track_wheel_info(node,full_param_name(parameters_ns,sprocket_wheel_param_name));
  declare_track_wheel_info(node,full_param_name(parameters_ns,idler_wheel_param_name));
}

//-----------------------------------------------------------------------------
template<>
ContinuousTrack get_continuous_track_info<ContinuousTrack>(std::shared_ptr<rclcpp::Node> node,
                                                           const std::string & parameters_ns)
{
  return {get_continuous_track_common_info(node,parameters_ns),
        get_track_wheel_info(node,full_param_name(parameters_ns,sprocket_wheel_param_name)),
        get_track_wheel_info(node,full_param_name(parameters_ns,idler_wheel_param_name))};
}

//-----------------------------------------------------------------------------
template<>
void declare_continuous_track_info<TriangleContinuousTrack>(std::shared_ptr<rclcpp::Node> node,
                                                            const std::string & parameters_ns)
{
  declare_continuous_track_common_info(node,parameters_ns);
  declare_track_wheel_info(node,full_param_name(parameters_ns,sprocket_wheel_param_name));
  declare_track_wheel_info(node,full_param_name(parameters_ns,front_idler_wheel_param_name));
  declare_track_wheel_info(node,full_param_name(parameters_ns,rear_idler_wheel_param_name));
}

//-----------------------------------------------------------------------------
template<>
TriangleContinuousTrack get_continuous_track_info<TriangleContinuousTrack>(std::shared_ptr<rclcpp::Node> node,
                                                                           const std::string & parameters_ns)
{
  return {get_continuous_track_common_info(node,parameters_ns),
        get_track_wheel_info(node,full_param_name(parameters_ns,sprocket_wheel_param_name)),
        get_track_wheel_info(node,full_param_name(parameters_ns,front_idler_wheel_param_name)),
        get_track_wheel_info(node,full_param_name(parameters_ns,rear_idler_wheel_param_name))};
}

//-----------------------------------------------------------------------------
void declare_wheeled_axle_info(std::shared_ptr<rclcpp::Node> node,
                               const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,wheels_distance_param_name);
  declare_wheel_info(node,full_param_name(parameters_ns,wheels_param_name));
}

//-----------------------------------------------------------------------------
WheeledAxle get_wheeled_axle_info(std::shared_ptr<rclcpp::Node> node,
                                  const std::string & parameters_ns)
{
  return {get_parameter<double>(node,parameters_ns,wheels_distance_param_name),
        get_wheel_info(node,full_param_name(parameters_ns,wheels_param_name))};
}

//-----------------------------------------------------------------------------
template <typename Track>
void declare_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,tracks_distance_param_name);
  declare_continuous_track_info<Track>(node,full_param_name(parameters_ns,tracks_param_name));
}

//-----------------------------------------------------------------------------
template <typename Track>
ContinuousTrackedAxle<Track> get_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                                              const std::string & parameters_ns)
{
  return {get_parameter<double>(node,parameters_ns,tracks_distance_param_name),
        get_continuous_track_info<Track>(node,full_param_name(parameters_ns,tracks_param_name))};
}

//-----------------------------------------------------------------------------
void declare_two_wheeled_axles_info(std::shared_ptr<rclcpp::Node> node,
                                    const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,axle_distance_param_name);

  declare_wheeled_axle_info(
        node,full_param_name(parameters_ns,front_axle_param_name));

  declare_wheeled_axle_info(
        node,full_param_name(parameters_ns,rear_axle_param_name));

}

//-----------------------------------------------------------------------------
TwoWheeledAxles get_two_wheeled_axles_info(std::shared_ptr<rclcpp::Node> node,
                                           const std::string & parameters_ns)
{
  return {get_parameter<double>(node,parameters_ns,axle_distance_param_name),
        get_wheeled_axle_info(node,full_param_name(parameters_ns,front_axle_param_name)),
        get_wheeled_axle_info(node,full_param_name(parameters_ns,rear_axle_param_name))};

}


template void declare_continuous_tracked_axle_info<ContinuousTrack>(std::shared_ptr<rclcpp::Node> node,const std::string & parameters_ns);
template void declare_continuous_tracked_axle_info<TriangleContinuousTrack>(std::shared_ptr<rclcpp::Node> node,const std::string & parameters_ns);
template ContinuousTrackedAxle<ContinuousTrack> get_continuous_tracked_axle_info<ContinuousTrack>(std::shared_ptr<rclcpp::Node> node,const std::string & parameters_ns);
template ContinuousTrackedAxle<TriangleContinuousTrack> get_continuous_tracked_axle_info<TriangleContinuousTrack>(std::shared_ptr<rclcpp::Node> node,const std::string & parameters_ns);

}
