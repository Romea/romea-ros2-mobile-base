#ifndef romea_DeadReckoning_H
#define romea_DeadReckoning_H

//ros
#include <rclcpp/time.hpp>

//romea
#include <romea_core_mobile_base/kinematic/KinematicMeasure.hpp>

namespace romea{

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

}

#endif
