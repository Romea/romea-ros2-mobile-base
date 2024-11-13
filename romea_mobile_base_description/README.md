# romea_mobile_base_description

## 1) Overview ##

This package contains the description of gps sensors used in romea projects

## 2) Package organization ##

This package is organized into subdirectories as follows:

- config/ contains rviz configuration for robot urdf visualization

- launch/ contains launch files able to start robot urdf visualization

- python/ contains romea_gps_description python module able to load mobile configuration

- ros2_control/ contains ros2_control xacro definition for each supported vehicles

- urdf/ contains base descriptions of supported mobile vehicles  :
    - base1FASxxx .chassis.axcro :  base description for 1FAS2FWD, 1FAS2RWD  and 1FAS4WD vehicles 
    - base2ASxxx.chassis.xacro:  base description  for 2AS2FWD, 2AS2RWD  and 2AS4WD vehicles
    - base2FWSxxx.chassis.xacro:  base description  for 2FWS2FWD, 2FWS2RWD  and 2FWS4WD vehicles
    - base2TD.chassis.xacro:  base description  for tracked vehicles
    - base2THD.chassis.xacro:  base description  for high drive tracked vehicles
    - base2TTD.chassis.xacro:  base description  for tank vehicles
    - base4WD.chassis.xacro:  base description  for four wheel drive vehicles
    - base4WS4WD.chassis.xacro:  base description  four wheel steering vehicles
