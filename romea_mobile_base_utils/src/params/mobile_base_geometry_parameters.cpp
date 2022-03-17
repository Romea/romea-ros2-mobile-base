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

//-----------------------------------------------------------------------------
void declare_track_wheel_info(std::shared_ptr<rclcpp::Node> node,
                              const std::string & parameters_ns)
{
  romea::declare_parameter<double>(node,parameters_ns,radius_param_name);
  romea::declare_parameter<double>(node,parameters_ns,x_param_name);
  romea::declare_parameter_with_default<double>(node,parameters_ns,z_param_name,std::numeric_limits<double>::quiet_NaN());
}

//-----------------------------------------------------------------------------
void try_declare_track_wheel_info(std::shared_ptr<rclcpp::Node> node,
                                  const std::string & parameters_ns)
{
  try {
    declare_track_wheel_info(node,parameters_ns);
  }
  catch (...)
  {
  }
}

//-----------------------------------------------------------------------------
void declare_track_idlers_info(std::shared_ptr<rclcpp::Node> node,
                               const std::string & parameters_ns)
{
  try_declare_track_wheel_info(node,romea::full_param_name(parameters_ns,idler_wheel_param_name));
  try_declare_track_wheel_info(node,romea::full_param_name(parameters_ns,front_idler_wheel_param_name));
  try_declare_track_wheel_info(node,romea::full_param_name(parameters_ns,rear_idler_wheel_param_name));
}

//-----------------------------------------------------------------------------
void declare_track_rollers_info(std::shared_ptr<rclcpp::Node> node,
                                const std::string & parameters_ns)
{
  romea::declare_parameter<double>(node,parameters_ns,radius_param_name);
  romea::declare_vector_parameter<double>(node,parameters_ns,x_param_name);
  romea::declare_parameter_with_default<double>(node,parameters_ns,z_param_name,std::numeric_limits<double>::quiet_NaN());
}

//-----------------------------------------------------------------------------
romea::TrackWheel get_track_wheel_info(std::shared_ptr<rclcpp::Node> node,
                                       const std::string & parameters_ns)
{
  auto radius = romea::get_parameter<double>(node,parameters_ns,radius_param_name);
  auto x = romea::get_parameter<double>(node,parameters_ns,x_param_name);
  auto z = romea::get_parameter<double>(node,parameters_ns,z_param_name);
  z = std::isfinite(z) ? z : radius;
  return {radius,x,z};
}

//-----------------------------------------------------------------------------
std::optional<romea::TrackWheel> try_get_track_wheel_info(std::shared_ptr<rclcpp::Node> node,
                                                          const std::string & parameters_ns)
{
  try
  {
    return get_track_wheel_info(node,parameters_ns);
  }
  catch (...)
  {
    return {};
  }
}


//-----------------------------------------------------------------------------
std::vector<romea::TrackWheel> get_track_idlers_info(std::shared_ptr<rclcpp::Node> node,
                                                     const std::string & parameters_ns)
{
  auto idler_wheel = try_get_track_wheel_info(
        node,romea::full_param_name(parameters_ns,idler_wheel_param_name));

  if(idler_wheel)
  {
    return {*idler_wheel};
  }

  auto front_idler_wheel = try_get_track_wheel_info(
        node,romea::full_param_name(parameters_ns,front_idler_wheel_param_name));
  auto rear_idler_wheel = try_get_track_wheel_info(
        node,romea::full_param_name(parameters_ns,rear_idler_wheel_param_name));

  if( front_idler_wheel &&  rear_idler_wheel)
  {
    return {*front_idler_wheel,*rear_idler_wheel};
  }

  return {};
}

//-----------------------------------------------------------------------------
std::vector<romea::TrackWheel> get_track_rollers_info(std::shared_ptr<rclcpp::Node> node,
                                                      const std::string & parameters_ns)
{
  auto radius =romea::get_parameter<double>(node,parameters_ns,radius_param_name);
  auto x_vector = romea::get_vector_parameter<double>(node,parameters_ns,x_param_name);
  auto z = romea::get_parameter<double>(node,parameters_ns,z_param_name);
  z=  std::isfinite(z) ? z : radius;

  std::vector<romea::TrackWheel> roller_wheels;
  for(const double & x : x_vector)
  {
    romea::TrackWheel wheel = {radius,x,z};
    roller_wheels.push_back(wheel);
  }
  return roller_wheels;
}

}


namespace romea {

//-----------------------------------------------------------------------------
void declare_wheel_info(std::shared_ptr<rclcpp::Node> node,
                        const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,radius_param_name);
  declare_parameter<double>(node,parameters_ns,width_param_name);
  declare_parameter_with_default<double>(node,parameters_ns,hub_carrier_offset_param_name,0.0);
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
void declare_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                                   const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,width_param_name);
  declare_parameter<double>(node,parameters_ns,thickness_param_name);
  declare_track_wheel_info(node,full_param_name(parameters_ns,sprocket_wheel_param_name));
  declare_track_idlers_info(node,parameters_ns);
  declare_track_rollers_info(node,full_param_name(parameters_ns,rollers_param_name));
}

//-----------------------------------------------------------------------------
ContinuousTrack get_continuous_track_info(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  return {get_parameter<double>(node,parameters_ns,width_param_name),
        get_parameter<double>(node,parameters_ns,thickness_param_name),
        get_track_wheel_info(node,full_param_name(parameters_ns,sprocket_wheel_param_name)),
        get_track_idlers_info(node,parameters_ns),
        get_track_rollers_info(node,full_param_name(parameters_ns,rollers_param_name))};
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
void declare_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                          const std::string & parameters_ns)
{
  declare_parameter<double>(node,parameters_ns,tracks_distance_param_name);
  declare_continuous_track_info(node,full_param_name(parameters_ns,tracks_param_name));
}

//-----------------------------------------------------------------------------
ContinuousTrackedAxle get_continuous_tracked_axle_info(std::shared_ptr<rclcpp::Node> node,
                                                       const std::string & parameters_ns)
{
  return {get_parameter<double>(node,parameters_ns,tracks_distance_param_name),
        get_continuous_track_info(node,full_param_name(parameters_ns,tracks_param_name))};
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

}
