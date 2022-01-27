#include "romea_odo_utils/kinematic_factory.hpp"
#include <romea_core_common/math/Algorithm.hpp>

namespace
{
template <class KinematicParamaters>
void loadWheelbases(romea::NodeParameters & node_parameters,
                    KinematicParamaters & kinematic_parameters)
{

  double wheelBase = node_parameters.loadParam<double>("chassis.wheelbase");
  double bodyReferenceX = node_parameters.loadParam<double>("kinematic.body_reference_x");

  kinematic_parameters.frontWheelBase = wheelBase/2-bodyReferenceX;
  kinematic_parameters.rearWheelBase = wheelBase/2.+bodyReferenceX;
}
}

namespace romea {

//-----------------------------------------------------------------------------
void load_kinematic_params(NodeParameters & node_parameters,
                           SkidSteeringKinematic::Parameters & kinematic_parameters)
{

  std::string frameType = node_parameters.loadParam<std::string>("odometry.frame");

  if(frameType.compare("4WD")==0)
  {

    double frontTrack =  node_parameters.loadParam<double>("chassis.track.front")+ 2*node_parameters.loadParam<double>("chassis.hub_carrier_offset.front");
    double rearTrack =  node_parameters.loadParam<double>("chassis.track.rear") + 2*node_parameters.loadParam<double>("chassis.hub_carrier_offset.rear") ;
    kinematic_parameters.track=frontTrack;

    if(!romea::near(frontTrack,rearTrack))
    {
      throw(std::runtime_error("Cannot handle a 4WD vehicle with two differents track"));
    }

  }
  else if(frameType.compare("2WD")==0)
  {
    kinematic_parameters.track=node_parameters.loadParam<double>("chassis.track")+ 2*node_parameters.loadParam<double>("chassis.hub_carrier_offset");
  }
  else
  {
    throw(std::runtime_error("Odometry frame "+ frameType + " is unsupported"));
  }

  kinematic_parameters.maximalWheelSpeed=node_parameters.loadParam<double>("kinematic.maximal_wheel_speed");
  kinematic_parameters.maximalWheelAcceleration=node_parameters.loadParam<double>("kinematic.maximal_wheel_acceleration");
  kinematic_parameters.wheelSpeedVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_speed_std"),2);

}

//-----------------------------------------------------------------------------
void load_kinematic_params(NodeParameters & node_parameters,
                           OneAxleSteeringKinematic::Parameters & kinematic_parameters)
{

  loadWheelbases(node_parameters,kinematic_parameters);
  kinematic_parameters.frontTrack = node_parameters.loadParam<double>("chassis.track.front");
  kinematic_parameters.rearTrack = node_parameters.loadParam<double>("chassis.track.rear");
  kinematic_parameters.frontHubCarrierOffset = node_parameters.loadParam<double>("chassis.hub_carrier_offset.front");
  kinematic_parameters.rearHubCarrierOffset = node_parameters.loadParam<double>("chassis.hub_carrier_offset.rear");


  std::string frameType = node_parameters.loadParam<std::string>("odometry.frame");
  double maximalWheelSpeed = node_parameters.loadParam<double>("kinematic.maximal_wheel_speed");

  if(frameType.compare("1FAS2FWD")==0 ||
     frameType.compare("2FWS2FWD")==0)
  {
    kinematic_parameters.frontMaximalWheelSpeed =maximalWheelSpeed;
  }
  else if(frameType.compare("1FAS2RWD")==0 ||
          frameType.compare("2FWS2RWD")==0)
  {
    kinematic_parameters.rearMaximalWheelSpeed= maximalWheelSpeed;
  }
  else if(frameType.compare("1FAS4WD")==0 ||
          frameType.compare("2FWS4WD")==0)
  {
    kinematic_parameters.frontMaximalWheelSpeed =maximalWheelSpeed;
    kinematic_parameters.rearMaximalWheelSpeed= maximalWheelSpeed;
  }
  else
  {
    throw(std::runtime_error("Odometry frame "+ frameType + " is unsupported"));
  }

  kinematic_parameters.maximalWheelAcceleration=node_parameters.loadParam<double>("kinematic.maximal_wheel_acceleration");

  if(frameType.compare("1FAS2FWD")==0 ||
     frameType.compare("1FAS2RWD")==0 ||
     frameType.compare("1FAS4WD")==0)
  {
    kinematic_parameters.maximalSteeringAngle = node_parameters.loadParam<double>("kinematic.maximal_steering_angle");
    kinematic_parameters.maximalSteeringAngularSpeed =  node_parameters.loadParam<double>("kinematic.maximal_steering_angular_speed");
    kinematic_parameters.wheelSpeedVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_speed_std"),2);
    kinematic_parameters.steeringAngleVariance = std::pow(node_parameters.loadParam<double>("odometry.steering_angle_std"),2);

  }
  else
  {
    double halfTrack = kinematic_parameters.frontTrack/2.;
    double wheelbase = kinematic_parameters.frontWheelBase+kinematic_parameters.rearWheelBase;
    double maximalWheelAngle = node_parameters.loadParam<double>("kinematic.maximal_wheel_angle");
    double maximalInstantaneousCurvature = TwoWheelSteeringKinematic::
        computeMaximalInstantaneousCurvature(wheelbase,halfTrack,maximalWheelAngle);

#warning add computation of parameters.maximalSteeringAngularSpeed
#warning add computation of parameters.steeringAngleVariance
    kinematic_parameters.maximalSteeringAngle = std::atan(maximalInstantaneousCurvature*wheelbase);
  }

  kinematic_parameters.wheelSpeedVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_speed_std"),2);

}


//-----------------------------------------------------------------------------
void load_kinematic_params(NodeParameters & node_parameters,
                           TwoAxleSteeringKinematic::Parameters & kinematic_parameters)
{

  loadWheelbases(node_parameters,kinematic_parameters);
  kinematic_parameters.frontTrack= node_parameters.loadParam<double>("chassis.track.front");
  kinematic_parameters.rearTrack= node_parameters.loadParam<double>("chassis.track.rear");
  kinematic_parameters.frontHubCarrierOffset=node_parameters.loadParam<double>("chassis.hub_carrier_offset.front");
  kinematic_parameters.rearHubCarrierOffset= node_parameters.loadParam<double>("chassis.hub_carrier_offset.rear");

  std::string frameType = node_parameters.loadParam<std::string>("odometry.frame");
  double maximalWheelSpeed = node_parameters.loadParam<double>("kinematic.maximal_wheel_speed");

  if(frameType.compare("2AS2FWD")==0)
  {
    kinematic_parameters.frontMaximalWheelSpeed =maximalWheelSpeed;
  }
  else if(frameType.compare("2AS2RWD")==0)
  {
    kinematic_parameters.rearMaximalWheelSpeed= maximalWheelSpeed;
  }
  else if(frameType.compare("2AS4WD")==0 ||
          frameType.compare("4WS4WD")==0)
  {
    kinematic_parameters.frontMaximalWheelSpeed =maximalWheelSpeed;
    kinematic_parameters.rearMaximalWheelSpeed= maximalWheelSpeed;
  }
  else
  {
    throw(std::runtime_error("Odometry frame "+ frameType + " is unsupported"));
  }

  kinematic_parameters.maximalWheelAcceleration = node_parameters.loadParam<double>("kinematic.maximal_wheel_acceleration");
  if(frameType.compare("4WS4WD")==0)
  {
#warning real maximal steering angles must be computed in this case
    kinematic_parameters.frontMaximalSteeringAngle=node_parameters.loadParam<double>("kinematic.maximal_wheel_angle");
    kinematic_parameters.rearMaximalSteeringAngle = kinematic_parameters.frontMaximalSteeringAngle;
  }
  else
  {
    kinematic_parameters.frontMaximalSteeringAngle = node_parameters.loadParam<double>("kinematic.front_maximal_steering_angle");
    kinematic_parameters.rearMaximalSteeringAngle = node_parameters.loadParam<double>("kinematic.rear_maximal_steering_angle");
    kinematic_parameters.maximalSteeringAngularSpeed = node_parameters.loadParam<double>("kinematic.maximal_steering_angular_speed");
    kinematic_parameters.wheelSpeedVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_speed_std"),2);
    kinematic_parameters.steeringAngleVariance = std::pow(node_parameters.loadParam<double>("odometry.steering_angle_std"),2);
  }
}


//-----------------------------------------------------------------------------
void load_kinematic_params(NodeParameters & node_parameters,
                           TwoWheelSteeringKinematic::Parameters & kinematic_parameters)
{

  loadWheelbases(node_parameters,kinematic_parameters);
  kinematic_parameters.frontTrack= node_parameters.loadParam<double>("chassis.track.front");
  kinematic_parameters.rearTrack= node_parameters.loadParam<double>("chassis.track.rear");
  kinematic_parameters.frontHubCarrierOffset=node_parameters.loadParam<double>("chassis.hub_carrier_offset.front");
  kinematic_parameters.rearHubCarrierOffset= node_parameters.loadParam<double>("chassis.hub_carrier_offset.rear");


  std::string frameType = node_parameters.loadParam<std::string>("odometry.frame");
  double maximalWheelSpeed = node_parameters.loadParam<double>("kinematic.maximal_wheel_speed");

  if(frameType.compare("2FWS2FWD")==0)
  {
    kinematic_parameters.frontMaximalWheelSpeed =maximalWheelSpeed;
  }
  else if(frameType.compare("2FWS2RWD")==0)
  {
    kinematic_parameters.rearMaximalWheelSpeed= maximalWheelSpeed;
  }
  else if(frameType.compare("2FWS4WD")==0)
  {
    kinematic_parameters.frontMaximalWheelSpeed =maximalWheelSpeed;
    kinematic_parameters.rearMaximalWheelSpeed= maximalWheelSpeed;
  }
  else
  {
    throw(std::runtime_error("Odometry frame "+ frameType + " is unsupported"));
  }

  kinematic_parameters.maximalWheelAcceleration=node_parameters.loadParam<double>("kinematic.maximal_wheel_acceleration");
  kinematic_parameters.maximalWheelAngle = node_parameters.loadParam<double>("kinematic.maximal_wheel_angle");
  kinematic_parameters.maximalWheelAngularSpeed = node_parameters.loadParam<double>("kinematic.maximal_wheel_angular_speed");
  kinematic_parameters.wheelSpeedVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_speed_std"),2);
  kinematic_parameters.wheelAngleVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_angle_std"),2);
}


//-----------------------------------------------------------------------------
void load_kinematic_params(NodeParameters & node_parameters,
                           FourWheelSteeringKinematic::Parameters & kinematic_parameters)
{

  loadWheelbases(node_parameters,kinematic_parameters);

  double frontTrack= node_parameters.loadParam<double>("chassis.track.front");
  double rearTrack= node_parameters.loadParam<double>("chassis.track.rear");
  kinematic_parameters.track= frontTrack;

  if(!romea::near(frontTrack,rearTrack))
  {
    throw(std::runtime_error("Cannot handle a 4WS4WD vehicle with two differents track"));
  }

  double frontHubCarrierOffset=node_parameters.loadParam<double>("chassis.hub_carrier_offset.front");
  double rearHubCarrierOffset= node_parameters.loadParam<double>("chassis.hub_carrier_offset.rear");
  kinematic_parameters.hubCarrierOffset=frontHubCarrierOffset;

  if(!romea::near(frontHubCarrierOffset,rearHubCarrierOffset))
  {
    throw(std::runtime_error("Cannot handle a 4WS4WD vehicle with different hub carrier offsets"));
  }


  std::string frameType = node_parameters.loadParam<std::string>("odometry.frame");
  if(frameType.compare("4WS4WD")!=0)
  {
    throw(std::runtime_error("Odometry frame "+ frameType + " is unsupported"));
  }

  kinematic_parameters.maximalWheelSpeed=node_parameters.loadParam<double>("kinematic.maximal_wheel_speed");
  kinematic_parameters.maximalWheelAcceleration = node_parameters.loadParam<double>("kinematic.maximal_wheel_acceleration");
  kinematic_parameters.maximalWheelAngle=node_parameters.loadParam<double>("kinematic.maximal_wheel_angle");
  kinematic_parameters.maximalWheelAngularSpeed = node_parameters.loadParam<double>("kinematic.maximal_wheel_angular_speed");
  kinematic_parameters.wheelSpeedVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_speed_std"),2);
  kinematic_parameters.wheelAngleVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_angle_std"),2);
}

//-----------------------------------------------------------------------------
void load_kinematic_params(NodeParameters & node_parameters,
                           MecanumWheelSteeringKinematic::Parameters & kinematic_parameters)
{

  std::string frameType = node_parameters.loadParam<std::string>("odometry.frame");

  if(frameType.compare("4MWD")==0)
  {

    double frontTrack =  node_parameters.loadParam<double>("chassis.track.front")+ 2*node_parameters.loadParam<double>("chassis.hub_carrier_offset.front");
    double rearTrack =  node_parameters.loadParam<double>("chassis.track.rear") + 2*node_parameters.loadParam<double>("chassis.hub_carrier_offset.rear") ;
    kinematic_parameters.track=frontTrack;

    if(!romea::near(frontTrack,rearTrack))
    {
      throw(std::runtime_error("Cannot handle a 4MWD vehicle with two differents track"));
    }

    kinematic_parameters.wheelbase = node_parameters.loadParam<double>("chassis.wheelbase");

  }
  else
  {
    throw(std::runtime_error("Odometry frame "+ frameType + " is unsupported"));
  }

  kinematic_parameters.maximalWheelSpeed=node_parameters.loadParam<double>("kinematic.maximal_wheel_speed");
  kinematic_parameters.maximalWheelAcceleration=node_parameters.loadParam<double>("kinematic.maximal_wheel_acceleration");
  kinematic_parameters.wheelSpeedVariance = std::pow(node_parameters.loadParam<double>("odometry.wheel_speed_std"),2);
}


//-----------------------------------------------------------------------------
void load_command_constraints(NodeParameters & node_parameters,
                              OneAxleSteeringConstraints &constraints)
{
  double minimal_linear_speed;
  if(node_parameters.loadParam("minimal_linear_speed",minimal_linear_speed))
  {
    constraints.setMinimalLinearSpeed(minimal_linear_speed);
  }
  else
  {
    constraints.setMinimalLinearSpeed(0.);
  }


  double maximal_linear_speed;
  if(node_parameters.loadParam("maximal_linear_speed",maximal_linear_speed))
  {
    constraints.setMaximalLinearSpeed(maximal_linear_speed);
  }

  double maximal_absolute_steering_angle;
  if(node_parameters.loadParam("maximal_steering_angle",maximal_absolute_steering_angle))
  {
    constraints.setMaximalAbsoluteSteeringAngle(maximal_absolute_steering_angle);
  }
}


//-----------------------------------------------------------------------------
void load_command_constraints(NodeParameters & node_parameters,
                              SkidSteeringConstraints & constraints)
{

  double minimal_linear_speed;
  if(node_parameters.loadParam("minimal_linear_speed",minimal_linear_speed))
  {
    constraints.setMinimalLinearSpeed(minimal_linear_speed);
  }
  else
  {
    constraints.setMinimalLinearSpeed(0.);
  }

  double maximal_linear_speed;
  if(node_parameters.loadParam("maximal_linear_speed",maximal_linear_speed))
  {
    constraints.setMaximalLinearSpeed(maximal_linear_speed);
  }

  double maximal_angular_speed;
  if(node_parameters.loadParam("maximal_angular_speed",maximal_angular_speed))
  {
    constraints.setMaximalAbsoluteAngularSpeed(maximal_angular_speed);
  }
}

//-----------------------------------------------------------------------------
void  load_command_constraints(NodeParameters & node_parameters,
                               TwoAxleSteeringConstraints & constraints)
{

  double minimal_linear_speed;
  if(node_parameters.loadParam("minimal_linear_speed",minimal_linear_speed))
  {
    constraints.setMinimalLinearSpeed(minimal_linear_speed);
  }
  else
  {
    constraints.setMinimalLinearSpeed(0.);
  }

  double maximal_linear_speed;
  if(node_parameters.loadParam("maximal_linear_speed",maximal_linear_speed))
  {
    constraints.setMaximalLinearSpeed(maximal_linear_speed);
  }

  double maximal_steering_angle;
  if(node_parameters.loadParam("maximal_steering_angle",maximal_steering_angle))
  {
    constraints.setMaximalAbsoluteFrontSteeringAngle(maximal_steering_angle);
    constraints.setMaximalAbsoluteRearSteeringAngle(maximal_steering_angle);
  }

  double front_maximal_steering_angle;
  if(node_parameters.loadParam("front_maximal_steering_angle",front_maximal_steering_angle))
  {
    constraints.setMaximalAbsoluteFrontSteeringAngle(front_maximal_steering_angle);
  }

  double rear_maximal_steering_angle;
  if(node_parameters.loadParam("rear_maximal_steering_angle",rear_maximal_steering_angle))
  {
    constraints.setMaximalAbsoluteRearSteeringAngle(rear_maximal_steering_angle);
  }

}

//-----------------------------------------------------------------------------
void load_command_constraints(NodeParameters & node_parameters,
                              OmniSteeringConstraints & constraints)
{
  double minimal_linear_speed;
  if(node_parameters.loadParam("minimal_longitudinal_speed",minimal_linear_speed))
  {
    constraints.setMinimalLongitudinalSpeed(minimal_linear_speed);
  }
  else
  {
    constraints.setMinimalLongitudinalSpeed(0.);
  }

  double maximal_linear_speed;
  if(node_parameters.loadParam("minimal_longitudinal_speed",maximal_linear_speed))
  {
    constraints.setMaximalLongitudinalSpeed(maximal_linear_speed);
  }

  double maximal_lateral_speed;
  if(node_parameters.loadParam("maximal_lateral_speed",maximal_lateral_speed))
  {
    constraints.setMinimalAbsoluteLateralSpeed(maximal_linear_speed);
  }


  double maximal_angular_speed;
  if(node_parameters.loadParam("maximal_steering_angle",maximal_angular_speed))
  {
    constraints.setMaximalAbsoluteAngularSpeed(maximal_angular_speed);
  }
}

}
