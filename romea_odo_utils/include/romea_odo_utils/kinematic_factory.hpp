#ifndef _romea_KinematicFactory_hpp_
#define _romea_KinematicFactory_hpp_

//romea
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_core_odo/kinematic/wheel_steering/FourWheelSteeringKinematic.hpp>
#include <romea_core_odo/kinematic/wheel_steering/TwoWheelSteeringKinematic.hpp>
#include <romea_core_odo/kinematic/axle_steering/OneAxleSteeringKinematic.hpp>
#include <romea_core_odo/kinematic/axle_steering/TwoAxleSteeringKinematic.hpp>
#include <romea_core_odo/kinematic/skid_steering/SkidSteeringKinematic.hpp>
#include <romea_core_odo/kinematic/omni_steering/MecanumWheelSteeringKinematic.hpp>

#include <romea_core_odo/kinematic/axle_steering/OneAxleSteeringConstraints.hpp>
#include <romea_core_odo/kinematic/axle_steering/TwoAxleSteeringConstraints.hpp>
#include <romea_core_odo/kinematic/skid_steering/SkidSteeringConstraints.hpp>

namespace romea {

void load_kinematic_params(NodeParameters & node_parameters,
                         SkidSteeringKinematic::Parameters & kinematic_parameters);

void load_kinematic_params(NodeParameters & node_parameters,
                         OneAxleSteeringKinematic::Parameters & kinematic_parameters);

void load_kinematic_params(NodeParameters & node_parameters,
                         TwoAxleSteeringKinematic::Parameters & kinematic_parameters);

void load_kinematic_params(NodeParameters & node_parameters,
                         TwoWheelSteeringKinematic::Parameters & kinematic_parameters);

void load_kinematic_params(NodeParameters & node_parameters,
                         FourWheelSteeringKinematic::Parameters & kinematic_parameters);

void load_kinematic_params(NodeParameters & node_parameters,
                         FourWheelSteeringKinematic::Parameters & kinematic_parameters);

void load_kinematic_params(NodeParameters & node_parameters,
                         MecanumWheelSteeringKinematic::Parameters & kinematic_parameters);

template <typename Params>
Params load_kinematic_params(NodeParameters & node_parameters)
{
  Params parameters;
  load_kinematic_params(node_parameters,parameters);
  return parameters;
}

void load_command_constraints(NodeParameters & node_parameters,
                            OneAxleSteeringConstraints & constraints);

void load_command_constraints(NodeParameters & node_parameters,
                            SkidSteeringConstraints & constraints);

void load_command_constraints(NodeParameters & node_parameters,
                            TwoAxleSteeringConstraints & constraints);

void load_command_constraints(NodeParameters & node_parameters,
                            OmniSteeringConstraints & constraints);

template <typename Constraints>
Constraints load_command_constraints(NodeParameters & node_parameters)
{
  Constraints constraints;
  load_command_constraints(node_parameters,constraints);
  return constraints;
}



}

#endif
