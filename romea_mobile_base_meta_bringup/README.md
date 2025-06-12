# romea_mobile_base_bringup

## 1) Overview ##

The romea_mobile_base_bringup package provides  : 

 - **Launch files** able to launch ros2 mobile base drivers according a meta-description file provided by user (see next section for mobile base meta-description file overview). It is possible to launch robot via command line : 

    ```console
    ros2 launch romea_mobile_base_bringup robot.launch.py robot_namespace:=robot mode:=simulation meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *mode* is the demonstation mode (live or simulation)  
   - *meta_description_file_path* is the absolute path of meta-description file    

   **Be careful, when the simulation mode is chosen the simulator is not started by this launch script and must be launched separately.**

 - A **python module** able to load and parse mobile base meta-description file as well as to create URDF description of the robot according a given meta-description.

 - A **ROS2 python executable** able to create mobile base URDF description via command line according a given meta-description file  :

  ```console
  ros2 run romea_mobile_base_bringup urdf_description.py robot_namespace:robot  mode:simulation meta_description_file_path:/path_to_file/meta_description_file.yaml > base.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *mode* is the demonstation mode (live or simulation)  
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with sensor URDFs to create a complete URDF description of the robot.  


## 2) Mobile base meta-description ##

As seen below mobile base meta-description file is a yaml file constituted by four items. The first item is the name of the robot defined by user. The second item provides basics specifications of the robot. The third item gives the topics to be recorded into the ROS bag during experiments or simulation. Thanks to remappings written into launch files, mobile base topics are always the same names for robot. Last item provides the initial position and orientation      

Example :
```yaml
name: "base" # name of the base
configuration: # mobile base basic specifications
  type: adap2e # robot type
  model: fat # robot model (optional)
records:  # topics to be recorded
  joint_states: true # joint_states will be recorded into bag
  controller/odom: true  # controller/odom will be recorded into bag
  controller/odometry: false  # controller/odometry will not be recorded into bag
  controller/kinematic: true  # controller/kinematic will be recorded into bag
simulation: # simulation configuration
  initial_xyz: [0.0, 0.0, 0.0] # initial position of the robot in simulation world
  initial_rpy: [0.0, 0.0, 0.0] # initial oriention of the robot in simulation world
```

### 3) Supported robot models

Supported robot are listed in the following table :

|  type  |   model    |
| :----: | :--------: |
| adap2e ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/adap2e), [gihub](https://github.com/Romea/adap2e)) |    fat     |
| adap2e ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/adap2e), [gihub](https://github.com/Romea/adap2e)) |    slim    |
| alpo ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/alpo), [github](https://github.com/Romea/alpo)) | fat (4x4) |
| alpo ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/alpo), [github](https://github.com/Romea/alpo)) |   slim (pom)  |
| aroco ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/aroco), [github](https://github.com/Romea/aroco)) |    none    |
| campeo ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/campero), [github](https://github.com/Romea/campero)) |    rubber    |
| ceol ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/ceol), [github](https://github.com/Romea/ceol)) | none |
| cinteo ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/cinteo), [github](https://github.com/Romea/cinteo)) | none |
| effibote3 ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/effibote3), [github](https://github.com/Romea/effibote3)) |    none    |
| husky ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/husky), [github](https://github.com/Romea/husky)) | none |
| robucar ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/robucar), [github](https://github.com/Romea/robucar)) |    none    |
| scout ([gitlab](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/scout), [github](https://github.com/Romea/scout)) | mini |


You can find specifications of each robot in config directory of their description package.

