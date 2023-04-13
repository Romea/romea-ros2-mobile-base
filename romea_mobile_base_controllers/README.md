# 1 Overview #

This package provides controllers for different kinds of vehicle as :
- skid steering vehicles:
    - 2 wheel drive vehicle (2WD)   **NOT TESTED**
    - 4 wheel drive vehicle (4WD)
    - 2 track drive vehicle (2TD, 2THD, 2TTD)    **NOT TESTED**
    
- one axle steering vehicles:
    - 1 front stering axle + 2 rear wheel drive vehicle (1FAS2RWD)  **NOT TESTED**
    - 1 front stering axle + 2 front wheel drive vehicle (1FAS2FWD)  **NOT TESTED**
    - 1 front stering axle + 4 wheel drive vehicle (1FAS4WD)  **NOT YET IMPLEMENTED**
    
- two axle steering vehicles:
    - 2 stering axles + 2 front wheel drive vehicle (2AS2FWD) **NOT YET IMPLEMENTED**
    - 2 stering axles + 2 front wheel drive vehicle (2AS2RWD) **NOT YET IMPLEMENTED**
    - 2 stering axles + 4 rear wheel drive vehicle (2AS4WD)
    
- two wheel steering vehicles
    - 2 front wheel steering + 4 wheel drive vehicle (2FWS4WD)
    - 2 front wheel steering + 2 front wheel drive vehicle (2FWS2FWD)   **NOT TESTED**
    - 2 front wheel steering + 2 rear wheel drive vehicle (2FWS2RWD)
    
- four wheel steering vehicles
    - 4 wheel steering + 4 wheel drive vehicle (4WS4WD) 



# 2 Controllers common API #

### 2.1 Published Topics ###

- odom (nav_msgs/Odometry)

  Dead reckoning localisation info

- kinematic (romea_mobile_base_msgs/KinematicMeasureStamped)

  Kinematic information (2D twist + instantaneaous curvature ) estimated from raw odometry data 

### 2.1 Parameters

- controller.base_frame_id (string, default base_link)

  Name of base frame id 

- controller.odom_frame_id (string, default odom)

  Name of odom frame id 

- controller.enable_odom_tf (bool, default false)

  Enable to publish TF transform from odom  

- controller.publish_rate (double, default 50.0) 

  Rate at which odometry data are published

- "controller.timeout (double, default 0.5)

  Time delay (in second) before the controller sends null command to robot actuators when it is not receiving any command from control nodes

- controller.joints_prefix(string, none)

  Prefix using in robot URDF description for each joints

- cmd_range.maximal_linear_speed.slow_mode (double, default none)

  Output maximal linear speed when slow mode is activated   

- base_info (dictionnary, default none)

  Geometry and mechanicals information coming from description package of the the robot to be controlled  

# 3 Controllers specific API #

## 3.1 skid steering controller family (2WD,4WD and 2TD) ##

### 3.1.1 Subscribed Topics ###

- cmd_skid_steering : (romea_mobile_base_msgs/SkidSteeringCommand)

  Command messages send by teleop or control nodes.  

### 3.1.2 Published Topics ###

- odometry (romea_mobile_base_msgs/SkidSteeringMeasureStamped)

  Kinematic information provided in the same operating space than the command 

### 3.1.2 Parameters

- controller.command_limits.minimal_longitudinal_speed: (double, default -max)

  Minimal longitudinal speed, must be lower or equal than 0

- controller.command_limits.maximal_longitudinal_speed: (double, default max)

  Maximal longitudinal speed, must be greater or equal than 0

- controller.command_limits. maximal_angular_speed: (double, default max)

  Absolute maximal angular speed

## 3.2 One axle steering and two wheels steering controller family (1FAS2RWD, 1FAS2FWD, 2FWS4WD, 2FWS2RWD and 2FWS2FWD) ##

### 3.2.1 Subscribed Topics ###

- cmd_two_axle_steering : (romea_mobile_base_msgs/TwoAxleSteeringCommand)

  Command messages send by teleop or control nodes.  

### 3.2.2 Published Topics ###

- odometry (romea_mobile_base_msgs/TwoAxleSteeringMeasureStamped)

  Kinematic information provided in the same operating space than the command 

### 3.2.2 Parameters

- controller.command_limits.minimal_longitudinal_speed: (double, default -max)

  Minimal longitudinal speed, must be lower or equal than 0

- controller.command_limits.maximal_longitudinal_speed: (double, default max)

  Maximal longitudinal speed, must be greater or equal than 0

- controller.command_limits.steering_angle: (double, default pi/2)

  Absolute maximal steering angle

## 3.2 Two axle steering and four wheels steering controller family (2AS4WD and 4WS4WD) ##

### 3.2.1 Subscribed Topics ###

- cmd_two_axle_steering : (romea_mobile_base_msgs/TwoAxleSteeringCommand)

  Command messages send by teleop or control nodes.  

### 3.2.2 Published Topics ###

- odometry (romea_mobile_base_msgs/TwoAxleSteeringMeasureStamped)

  Kinematic information provided in the same operating space than the command 

### 3.2.2 Parameters

- controller.command_limits.minimal_longitudinal_speed: (double, default -max)

  Minimal longitudinal speed, must be lower or equal than 0

- controller.command_limits.maximal_longitudinal_speed: (double, default max)

  Maximal longitudinal speed, must be greater or equal than 0

- controller.command_limits.front_steering_angle: (double, default pi/2)

  Absolute front maximal steering angle

- controller.command_limits.rear_steering_angle: (double, default pi/2)

  Absolute front maximal steering angle

