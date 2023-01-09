// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_HPP_
#define ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_HPP_

// ros
#include <rclcpp/time.hpp>

// romea core
#include <romea_core_mobile_base/kinematic/KinematicMeasure.hpp>

// std
#include <optional>

namespace romea
{

class DeadReckoning
{
public:
  DeadReckoning();

  void update(const rclcpp::Time & time, const KinematicMeasure & kinematic_measure);

  const double & getX()const;

  const double & getY()const;

  const double & getHeading()const;

  void reset();

private:
  std::optional<rclcpp::Time> previous_update_time_;

  double x_;
  double y_;
  double heading_;
  double previous_longitudinal_speed_;
  double previous_lateral_speed_;
  double previous_angular_speed_;
};

}  // namespace romea

#endif  // ROMEA_MOBILE_BASE_CONTROLLERS__DEAD_RECKONING_HPP_
