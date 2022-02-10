#include "romea_mobile_base_utils/params/mobile_base_geometry_parameters.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace
{
const std::string radius_param_name="radius";
const std::string width_param_name="width";
const std::string hub_carrier_offset_param_name="hub_carrier_offset";

const std::string wheels_param_name="wheels";
const std::string wheels_distance_param_name="wheels_distance";

const std::string tracks_param_name="tracks";
const std::string tracks_distance_param_name="tracks_distance";

const std::string front_axle_param_name="front_axle";
const std::string rear_axle_param_name="rear_axle";
const std::string axle_distance_param_name="axles_distance";

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
void get_wheel_info(std::shared_ptr<rclcpp::Node> node,
                    const std::string & parameters_ns,
                    Wheel & wheel_info)
{

  wheel_info.radius = get_parameter<double>(node,parameters_ns,radius_param_name);
  wheel_info.width = get_parameter<double>(node,parameters_ns,width_param_name);
  wheel_info.hubCarrierOffset = get_parameter<double>(
        node,parameters_ns,hub_carrier_offset_param_name);

}

//-----------------------------------------------------------------------------
void declare_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,width_param_name);

}

//-----------------------------------------------------------------------------
void get_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                               const std::string & parameters_ns,
                               ContinuousTrack & track_info)
{
  track_info.width = get_parameter<double>(
        node,parameters_ns,width_param_name);
}

//-----------------------------------------------------------------------------
void declare_wheeled_axle_info(std::shared_ptr<rclcpp::Node> node,
                               const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,wheels_distance_param_name);
  declare_wheel_info(node,full_param_name(parameters_ns,wheels_param_name));
}

//-----------------------------------------------------------------------------
void get_wheeled_axle_info(std::shared_ptr<rclcpp::Node> node,
                           const std::string & parameters_ns,
                           WheeledAxle & axle_info)
{
  axle_info.wheelsDistance = get_parameter<double>(
        node,parameters_ns,wheels_distance_param_name);

  get_wheel_info(node,full_param_name(parameters_ns,wheels_param_name),
                 axle_info.wheels);
}

//-----------------------------------------------------------------------------
void declare_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,tracks_distance_param_name);
  declare_continuous_track_info(node,full_param_name(parameters_ns,tracks_param_name));
}


//-----------------------------------------------------------------------------
void get_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                      const std::string & parameters_ns,
                                      ContinuousTrackedAxle & axle_info)
{
  axle_info.tracksDistance = get_parameter<double>(
        node,parameters_ns,tracks_distance_param_name);

  get_continuous_track_info(node,full_param_name(parameters_ns,tracks_param_name),
                            axle_info.tracks);
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
void get_two_wheeled_axles_info(std::shared_ptr<rclcpp::Node> node,
                                const std::string & parameters_ns,
                                TwoWheeledAxles & axles_info)
{
  axles_info.axlesDistance = get_parameter<double>(
        node,parameters_ns,axle_distance_param_name);

  get_wheeled_axle_info(
        node,full_param_name(parameters_ns,front_axle_param_name),
        axles_info.frontAxle);

  get_wheeled_axle_info(
        node,full_param_name(parameters_ns,rear_axle_param_name),
        axles_info.rearAxle);

}

}
