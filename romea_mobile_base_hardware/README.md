This package provides hardware interface for different kinds of vehicle :
- skid steering vehicles:
    - 2 wheel drive vehicle (2WD)   **NOT TESTED**
    - 4 wheel drive vehicle (4WD)
    - 2 track drive vehicle (2TD, 2THD, 2TTD)    **NOT TESTED**
    
- one axle steering vehicles:
    - 1 front stering axle + 2 rear wheel drive vehicle (1FAS2RWD)  **NOT TESTED**
    - 1 front stering axle + 2 front wheel drive vehicle (1FAS2FWD)  **NOT TESTED**
    - 1 front stering axle + 4 wheel drive vehicle (1FAS4WD)  **NOT TESTED**
    
- two axle steering vehicles:
    - 2 stering axles + 2 front wheel drive vehicle (2AS2FWD) **NOT TESTED**
    - 2 stering axles + 2 front wheel drive vehicle (2AS2RWD) **NOT TESTED**
    - 2 stering axles + 4 rear wheel drive vehicle (2AS4WD)
    
- two wheel steering vehicles
    - 2 front wheel steering + 4 wheel drive vehicle (2FWS4WD)
    - 2 front wheel steering + 2 front wheel drive vehicle (2FWS2FWD)   **NOT TESTED**
    - 2 front wheel steering + 2 rear wheel drive vehicle (2FWS2RWD)
    
- four wheel steering vehicles
    - 4 wheel steering + 4 wheel drive vehicle (4WS4WD) 

These interfaces are based on [ros2 hardware interface](https://github.com/ros-controls/ros2_control/tree/master/hardware_interface) and are able to get command from controller and send odometry data to controller.
