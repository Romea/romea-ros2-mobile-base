#include "romea_mobile_base_controllers/dead_reckoning.hpp"
#include <romea_core_common/math/EulerAngles.hpp>

namespace romea {

//-----------------------------------------------------------------------------
DeadReckoning::DeadReckoning():
  previous_update_time_(),
  x_(0),
  y_(0),
  heading_(0),
  previous_longitudinal_speed_(0),
  previous_lateral_speed_(0),
  previous_angular_speed_(0)
{

}

//-----------------------------------------------------------------------------
void DeadReckoning::reset()
{
  previous_update_time_.reset();
  x_=0;
  y_=0;
  heading_=0;
  previous_longitudinal_speed_=0;
  previous_lateral_speed_=0;
  previous_angular_speed_=0;
}

//-----------------------------------------------------------------------------
void DeadReckoning::update(const rclcpp::Time& time, const KinematicMeasure & kinematic_measure)
{
  if(previous_update_time_.has_value())
  {
    double dt = (time - *previous_update_time_).seconds();
    x_=x_+previous_longitudinal_speed_*dt;
    y_=y_+previous_lateral_speed_*dt;
    heading_= between0And2Pi(heading_+previous_angular_speed_*dt);
  }

  previous_longitudinal_speed_=kinematic_measure.longitudinalSpeed;
  previous_lateral_speed_=kinematic_measure.lateralSpeed;
  previous_angular_speed_=kinematic_measure.angularSpeed;
  previous_update_time_=time;
}

//-----------------------------------------------------------------------------
const double & DeadReckoning::getX()const
{
  return x_;
}

//-----------------------------------------------------------------------------
const double & DeadReckoning::getY()const
{
  return y_;
}

//-----------------------------------------------------------------------------
const double & DeadReckoning::getHeading()const
{
  return heading_;
}

}
