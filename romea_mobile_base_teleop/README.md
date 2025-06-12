# ROMEA Teleop Drivers #

# 1 Overview #

This package contains teleop nodes for skid, ackermann (one axle), four wheel (two axles) and omni steering robots. It converts joystick messages into command messages compatible with the vehicle controllers. Additionally, these nodes can automatically register and unregister the teleoperation output topic with the romea_cmd_mux_node from the romea_cmd_mux package.

# 2 Nodes #


### 2.1 skid_steering_teleop_node ###

### 2.1.1 Subscribed Topics ###

- **joy** : (sensor_msgs/Joy)

  Joystick messages to be translated to velocity commands.  

### 2.1.2 Published Topics ###

- **cmd_vel** (geometry_msgs/Twist)

  ROS velocity messages arising from joystick commands. Available if parameter cmd_output.message_type is equal to geometry_msgs/Twist.

- cmd_skid_steering (romea_mobile_base_msgs/SkidSteeringCommand)

  Romea skid steering command messages arising from joystick commands. Available if parameter cmd_output.message_type is equal to romea_mobile_base_msgs/SkidSteeringCommand.

### 2.1.3 Parameters ###

- jostick_mapping.axes.linear_speed (string, default node)

    Id of the stick (axe) used to get linear speed from joy msg

- jostick_maping.axes.angular_speed (string, default none)

    Id of the stick (axe) used to get angular speed from joy msg

- joystick_remapping.buttons.slow_mode (string, default none)

    Id of the button used to get slow motion mode status from joy msg

- joystick_remapping.buttons.turbo_mode (string, default none) 

    Id of the button used to get turbo motion mode status from joy msg

- cmd_output.message_type (string, default none)

  Type of messages published by teleop node : geometry_msgs/Twist or   romea_mobile_base_msgs/SkidSteeringCommand

- cmd_output.message_priority(int, default -1)

  Priority of messages from 0 to 255. If this parameters is defined by user, the teleop will call automatically service cmd_mux/subscribe to register output topic (cmd_vel or cmd_skid_steering) to cmd_mux node ( see romea_cmd_mux package). 

- cmd_range.maximal_linear_speed.slow_mode (double, default none)

    Output maximal linear speed when slow mode is activated   

- cmd_range.maximal_linear_speed.turbo_mode (double, default none)

    Output maximal linear speed when turbo mode is activated 

- cmd_range.maximal_angular_speed.slow_mode (double, default none)

    Output maximal angular speed  when slow motion mode is activated   

- cmd_range.maximal_angular_speed_.turbo_mode (double, default none)

    Output maximal angular speed  when turbo motion mode is activated      


# 2.2 omni_steering_teleop_node #

### 2.2.1 Subscribed Topics ###

  - joy : (sensor_msgs/Joy)

    Joystick messages to be translated to velocity commands.  

### 2.2.2 Published Topics ###

  - cmd_vel (geometry_msgs/Twist)

    Ros velocity messages arising from joystick commands. Available if parameter cmd_output.message_type is equal to geometry_msgs/Twist.
    
- cmd_omni_steering (romea_mobile_base_msgs/OmniSteeringCommand)

  Romea omni steering command messages arising from joystick commands. Available if parameter cmd_output.message_type is equal to romea_mobile_base_msgs/OmniSteeringCommand.


### 2.2.3 Parameters ###

- joystick_mapping.axes.linear_speed (string, default none)

     Id of the stick (axe) used to get linear speed from joy msg

- jostick_mapping.axes.lateral_speed (string, default none)

     Id of the stick (axe) used to get lateral speed from joy msg 

- jostick_mapping.axes.angular_speed (string, default none)

     Id of the stick (axe) used to get angular speed from joy msg 

- joystick.remapping.buttons.slow_mode (string, default none)

     Id of the button used to get slow motion mode status from joy msg 

- joystick.remapping.buttons.turbo_mode (string, default none) 

     Id of the button used to get turbo motion mode status from joy msg 

- cmd_output.message_type (string, default none)

     Type of messages published by teleop node : geometry_msgs/Twist or   romea_mobile_base_msgs/OmniSteeringCommand

- cmd_output.message_priority(int, default -1)

     Priority of messages from 0 to 255. If this parameters is defined by user, the teleop will call automatically service cmd_mux/subscribe to register output topic (cmd_vel or cmd_omni_steering) to cmd_mux node ( see romea_cmd_mux package). 

- cmd_range.maximal_linear_speed.slow_mode (double, default none)

     Output maximal linear speed when slow mode is activated   

- cmd_range.maximal_linear_speed.turbo_mode (double, default none)

     Output maximal linear speed when turbo mode is activated 

- cmd_range.maximal_lateral_speed.slow_mode (double, default none)

     Output maximal lateral speed when slow mode is activated   

- cmd_range.maximal_lateral_speed.turbo_mode (double, default none)

     Output maximal linear speed when turbo mode is activated 

- cmd_range.maximal_angular_speed.slow_mode (double, default none)

     Output maximal angular speed  when slow motion mode is activated   

- cmd_range.maximal_angular_speed_.turbo_mode (double, default none)

     Output maximal angular speed  when turbo motion mode is activated      


# 2.3 one_axle_teleop_node #

### 2.3.1 Subscribed Topics ###

  - joy : (sensor_msgs/Joy)

    Joystick messages to be translated to velocity commands.  

### 2.3.2 Published Topics ###

- cmd_vel (geometry_msgs/Twist)    

    Ros velocity messages arising from Joystick commands. To ensure compatibility with ackermann controller from ros_controllers package geometry_msgs::Twist msg is used to command instead of an ackermann msg. Linear speed is set into twist.linear.x parameter and steering angle is set into twist.angular.z parameter (yes it is weird) .  Available if parameter cmd_output.message_type is equal to (geometry_msgs/Twist). 

- cmd_omni_steering (romea_mobile_base_msgs/AxleSteeringCommand)

  Romea omni steering command messages arising from joystick commands. Available if parameter cmd_output.message_type is equal to romea_mobile_base_msgs/OneAxleSteeringCommand.

### 2.4.3 Parameters ###

  - jostick_mapping.axes.linear_speed (string, default none)

    Id of the stick (axe) used to get linear_speed from joy msg 

- jostick_mapping.axes.stering_angle (string, default none)

  Id of the stick (axe) used to get steering_angle from joy msg 

- joystick_mapping.buttons.slow_mode (string, default none)

  Id of the button (axe) used to get slow motion mode status from joy msg 

- joystick_mapping.buttons.turbo_mode (string, default none) 

  Id of the button (axe) used to get turbo motion mode status from joy msg 

- cmd_output.message_type (string, default none)

  Type of messages published by teleop node : geometry_msgs/Twist or   romea_mobile_base_msgs/OneAxleSteeringCommand

- cmd_output.message_priority(int, default -1)

  Priority of messages from 0 to 255. If this parameters is defined by user, the teleop will call automatically service cmd_mux/subscribe to register output topic (cmd_vel or cmd_one_axle_steering) to cmd_mux node ( see romea_cmd_mux package). 

- cmd_range.maximal_linear_speed.slow_mode (double, default none)

  Output maximal linear speed when slow mode is activated   

- cmd_range.maximal_linear_speed.turbo_mode (double, default none)

  Output maximal linear speed when turbo mode is activated 

- cmd_range.maximal_steering_angle (double, default none)

  Output maximal steering angle          

# 2.4 two_axle_steering_teleop_node #

### 2.4.1 Subscribed Topics ###

  - joy : (sensor_msgs/Joy)

    Joystick messages to be translated to velocity commands.  

### 2.4.2 Published Topics ###

  - cmd_4ws (four_wheel_steering_msgs/FourWheelSteering)

    Ros velocity messages arising from joystick commands. Available if parameter cmd_output.type is equal to four_wheel_steering_msgs/FourWheelSteering.

- cmd_two_axle_steering (romea_mobile_base_msgs/TwoAxleSteeringCommand)

  Romea two axle steering command messages arising from joystick commands. Available if parameter cmd_output.type is equal to romea_mobile_base_msgs/TwoAxleSteeringCommand.

### 2.4.3 Parameters ###

- jostick_mapping.axes.linear_speed (string, default none)

  Id of the stick (axe) used to get linear speed from joy msg 

- jostick_mapping.axes.front_stering_angle (string, default none)

  Id of the stick (axe) used to get front_steering_angle from joy msg 

- jostick_mapping.axes.rear_stering_angle (string, default none)

  Id of the stick (axe) used to get rear_steering_angle from joy msg 

- joystick_mapping.buttons.slow_mode (string, default none)

  Id of the button used to get slow motion mode status from joy msg 

- joystick_mapping.buttons.turbo_mode (string, default none) 

  Id of the button used to get turbo motion mode status from joy msg 

- cmd_output.message_type (string, default none)

  Type of messages published by teleop node : four_wheel_steering_msgs/FourWheelSteering or   romea_mobile_base_msgs/TwoAxleSteeringCommand

- cmd_output.message_priority(int, default -1)

  Priority of messages from 0 to 255. If this parameters is defined by user, the teleop will call automatically service cmd_mux/subscribe to register output topic (cmd_vel or cmd_one_axle_steering) to cmd_mux node ( see romea_cmd_mux package). 

- cmd_range.maximal_linear_speed.slow_mode (double, default none)

  Output maximal linear speed when slow mode is activated   

- cmd_range.maximal_linear_speed.turbo_mode (double, default none)

  Output maximal linear speed when turbo mode is activated 

- cmd_range.maximal_front_steering_angle (double, default none)

  Output maximal front steering angle          

- cmd_range.maximal_front_steering_angle (double, default none)

  Output maximal front steering angle 

