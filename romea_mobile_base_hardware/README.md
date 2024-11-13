# romea_mobile_base_hardware

This package provides generic hardware interface for different kinds of vehicle :

- **Skid steering vehicles**:
    - 2 wheel drive vehicle (2WD)   *not yet tested*
    - 4 wheel drive vehicle (4WD)
    - 2 track drive vehicle (2TD, 2THD, 2TTD)    *not yet tested*
    
- **One axle steering vehicles**:
    - 1 front steering axle + 2 rear wheel drive vehicle (1FAS2RWD)   *not yet tested*
    - 1 front stering axle + 2 front wheel drive vehicle (1FAS2FWD)   *not yet tested*
    - 1 front stering axle + 4 wheel drive vehicle (1FAS4WD)   *not yet tested*
    
- **Two axle steering vehicles**:
    - 2 steering axles + 2 front wheel drive vehicle (2AS2FWD)  *not yet tested*
    - 2 steering axles + 2 front wheel drive vehicle (2AS2RWD)  *not yet tested*
    - 2 steering axles + 4 rear wheel drive vehicle (2AS4WD)
    
- **Two wheel steering vehicles**
    - 2 front wheel steering + 4 wheel drive vehicle (2FWS4WD)
    - 2 front wheel steering + 2 front wheel drive vehicle (2FWS2FWD)    *not yet tested*
    - 2 front wheel steering + 2 rear wheel drive vehicle (2FWS2RWD)
    
- **Four wheel steering vehicles**
    - 4 wheel steering + 4 wheel drive vehicle (4WS4WD) 

These interfaces are built on top of the [ROS 2 Hardware Interface](https://github.com/ros-controls/ros2_control/tree/master/hardware_interface), enabling command reception from a controller and providing odometry data feedback. They serve as a foundational layer for developing specific hardware interfaces for each robot type by extending the base classes. Users only need to integrate communication code to send commands to the robot and relay odometry data back to the controller.
