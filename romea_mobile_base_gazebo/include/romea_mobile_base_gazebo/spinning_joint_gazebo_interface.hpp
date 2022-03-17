#ifndef _romea_SpinningJointGazeboInterface_hpp_
#define _romea_SpinningJointGazeboInterface_hpp_

#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"

#include <romea_mobile_base_hardware/spinning_joint_hardware_interface.hpp>

namespace romea
{

class SpinningJointGazeboInterface
{

public:

  enum CommandType {
    VELOCITY,
    EFFORT
  };

  struct Feedback
  {
    double position;
    double velocity;
    double effort;
  };

public :

  SpinningJointGazeboInterface(gazebo::physics::ModelPtr parent_model,
                               const hardware_interface::ComponentInfo & joint_info,
                               const std::string & command_interface_type);

  void setCommand(const double & command);

  Feedback getFeedback()const;

private:

  CommandType command_type;
  gazebo::physics::JointPtr sim_joint_;

};

void write(const SpinningJointHardwareInterface & hardware_joint,
           SpinningJointGazeboInterface & gazebo_joint);

void read(const SpinningJointGazeboInterface & gazebo_joint,
          SpinningJointHardwareInterface & hardware_joint);

void read(const SpinningJointGazeboInterface & gazebo_joint,
          SpinningJointHardwareInterface::Feedback & hardware_joint_feedback);

void read(const SpinningJointGazeboInterface::Feedback & gazebo_joint_feedback,
          SpinningJointHardwareInterface::Feedback & hardware_joint_feedback);


SpinningJointGazeboInterface::Feedback
drive_wheel_feedback(const SpinningJointGazeboInterface & drive_wheel_spinning_joint,
                     const SpinningJointGazeboInterface & idler_wheel_spinning_joint);

SpinningJointGazeboInterface::Feedback
drive_wheel_feedback(const SpinningJointGazeboInterface & high_drive_wheel_spinning_joint,
                     const SpinningJointGazeboInterface & front_ground_idler_wheel_spinning_joint,
                     const SpinningJointGazeboInterface & rear_ground_idler_wheel_spinning_joint,
                     const double & high_wheel_radius,
                     const double & ground_wheel_radius);
}
#endif
