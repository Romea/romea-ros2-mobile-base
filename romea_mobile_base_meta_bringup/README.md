# romea_mobile_base_meta_bringup

## 1) Overview

`romea_mobile_base_meta_bringup` provides tools to instantiate and launch a mobile base from a meta-description file.

It is built on top of `romea_common_meta_bringup` and specializes the meta-bringup workflow for the generic `romea_mobile_base` stack.

The package can generate consistent ROS 2 artifacts such as:

* mobile base configuration files
* mobile base URDF descriptions
* mobile base ros2_control descriptions
* mobile base launch files

The concrete robot implementation is not defined in this package. It is selected from the meta-description and delegated to the corresponding `<robot_name>_bringup` package.

---

## 2) Mobile base meta-description

### 2.1 Concept

A mobile base meta-description defines which concrete robot base must be used and how it must be launched in a robot system.

It does not contain the full geometry, inertia, URDF or hardware implementation of the robot. These details remain in robot-specific packages such as `<robot_name>_description`, `<robot_name>_hardware` and `<robot_name>_bringup`.

The meta-description centralizes:

* mobile base instance name
* selected robot model and optional version
* launch profiles to include
* topics to record

`configuration.model` selects the robot-specific bringup package. For example, `model: cinteo` delegates configuration, URDF and ros2_control generation to `cinteo_bringup`.

---

### 2.2 Format

The global structure of a mobile base meta-description is:

```yaml
name: <base_name>

configuration:
  manufacturer: <manufacturer_name>
  model: <robot_name>
  version: <robot_version>

launch:
  - include:
      file: <launch_profile>
      if: <launch_condition>

records:
  <topic_name>: <true_or_false>
```

The `model` field is used to import the `<robot_name>_bringup` Python module and to include launch files from the `<robot_name>_bringup` package.

The `version` field is optional. When present, it selects a robot variant supported by the robot-specific bringup package.

---

### 2.3 Example

```yaml
name: base

configuration:
  manufacturer: romea
  model: cinteo
  version: ""

launch:
  - include:
      file: "$(find-pkg-share romea_mobile_base_meta_bringup)/profile/robot.launch.py"
      if: $(eval "'$(var mode)' == 'live'")

records:
  joint_states: true
  controller/odom: true
  controller/odometry: false
  controller/kinematic: true
```

---

## 3) Generated artifacts

`romea_mobile_base_meta_bringup` delegates generation to the selected `<robot_name>_bringup` package.

The selected package must provide functions for generating:

* a compact mobile base configuration
* a mobile base URDF description
* a mobile base ros2_control description
* launch files that start the base and teleoperation stack

### 3.1 Generate configuration file

Generates a compact mobile base configuration from the robot-specific description package through the selected bringup package.

```bash
ros2 run romea_mobile_base_meta_bringup generate_configuration_file.py \
  meta_description_file_path:path/to/mobile_base_meta_description.yaml \
  extended:false
```

Example output (simplified):

```yaml
type: 1FAS4WD
command_type: one_axle_steering
command_limits:
  minimal_longitudinal_speed: -2.0
  maximal_longitudinal_speed: 2.0
  maximal_steering_angle: 0.7
inertia:
  mass: 250.0
wheelbase: 1.2
track: 0.8
```

---

### 3.2 Generate URDF description

Generates the URDF description associated with the selected robot model.

```bash
ros2 run romea_mobile_base_meta_bringup generate_urdf_description.py \
  robot_namespace:robot \
  meta_description_file_path:path/to/mobile_base_meta_description.yaml \
  mode:simulation_gazebo
```

The generated URDF contains the selected mobile base and can be combined with sensor, implement, and arm URDF descriptions to build a complete robot model.

Example output (simplified):

```xml
<link name="robot_base_link">
  ...
</link>

<joint name="robot_front_axle_steering_joint" type="revolute">
  <parent link="robot_base_link"/>
  <child link="robot_front_axle_steering_link"/>
  <origin xyz="..." rpy="..."/>
  ...
</joint>

<link name="robot_rear_left_wheel_link">
  <visual>
    ...
  </visual>
  <collision>
    ...
  </collision>
  <inertial>
    ...
  </inertial>
</link>

<joint name="robot_rear_left_wheel_spinning_joint" type="continuous">
  <parent link="robot_rear_left_wheel_steering_link"/>
  <child link="robot_rear_left_wheel_link"/>
  <origin xyz="..." rpy="..."/>
  ...
</joint>
```

---

### 3.3 Generate ros2_control description

Generates the ros2_control description associated with the selected robot model.

```bash
ros2 run romea_mobile_base_meta_bringup generate_ros2_control_description.py \
  robot_namespace:robot \
  meta_description_file_path:path/to/mobile_base_meta_description.yaml \
  mode:live
```

This description declares the hardware plugin, hardware parameters and joint command/state interfaces required by the selected mobile base architecture.

Example output (simplified):

```xml
<ros2_control name="robot_base" type="system">
  <hardware>
    <plugin>...</plugin>
    <param name="wheelbase">...</param>
    <param name="front_track">...</param>
    <param name="rear_track">...</param>
  </hardware>

  <joint name="robot_front_axle_steering_joint">
    <command_interface name="position">
      <param name="min">...</param>
      <param name="max">...</param>
    </command_interface>
    <state_interface name="position"/>
  </joint>

  <joint name="robot_rear_left_wheel_spinning_joint">
    <command_interface name="velocity">
      <param name="min">...</param>
      <param name="max">...</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

The exact hardware parameters and joint interfaces depend on the selected robot model and mobile base architecture.

---

### 3.4 Generate launch file

Generates a YAML ROS 2 launch file from the `launch` section of the mobile base meta-description.

```bash
ros2 run romea_mobile_base_meta_bringup generate_launch_file.py \
  robot_namespace:robot \
  meta_description_file_path:path/to/mobile_base_meta_description.yaml
```

The generated launch file:

* pushes the robot namespace and base namespace
* exposes configuration values as launch variables
* declares `mode`, `joystick_topic` and `joystick_configuration_file_path`
* includes the selected launch profiles

Example output (simplified):

```yaml
launch:
  - arg: {name: mode, default: live}
  - arg: {name: joystick_topic}
  - arg: {name: joystick_configuration_file_path}
  - group:
      - push-ros-namespace: {namespace: robot}
      - push-ros-namespace: {namespace: base}
      - let: {name: model, value: cinteo}
      - let: {name: tf_prefix, value: robot_}
      - let: {name: frame_id, value: robot_base_link}
      - include:
          file: $(find-pkg-share romea_mobile_base_meta_bringup)/profile/robot.launch.py
          if: $(eval "'$(var mode)' == 'live'")
```

---

## 4) Launch profiles

The `profile/` directory contains reusable launch files that can be included from a mobile base meta-description.

### 4.1 robot.launch.py

`robot.launch.py` is the standard profile used to start a mobile base.

It delegates to the selected robot-specific bringup package by including:

* `<robot_name>_bringup/launch/<robot_name>_base.launch.py`
* `<robot_name>_bringup/launch/<robot_name>_teleop.launch.py`

It also forwards the joystick topic and joystick configuration file path to the teleoperation launch file.

---

### 4.2 localisation_plugin.launch.py

`localisation_plugin.launch.py` starts the odometry localisation plugin associated with the mobile base controller feedback.

It can run either:

* as a standalone node
* as a composable node loaded into an existing container

This profile is optional and can be included from the `launch` section of the meta-description when odometry observations must be published to the robot localisation stack.

---

## 5) Usage and modes

The package provides one main launch file:

* `mobile_base.launch.py`

This launch file dynamically generates and includes the mobile base launch file from a mobile base meta-description.

```bash
ros2 launch romea_mobile_base_meta_bringup mobile_base.launch.py \
  mode:=live \
  robot_namespace:=robot \
  mobile_base_meta_description_file_path:=path/to/mobile_base_meta_description.yaml \
  joystick_meta_description_file_path:=path/to/joystick_meta_description.yaml
```

The mobile base launch workflow requires both:

* a mobile base meta-description
* a joystick meta-description

The joystick is considered part of the minimal mobile base control setup. `mobile_base.launch.py` loads the joystick meta-description to determine the joystick topic and the joystick message layout configuration, then forwards these values to the generated mobile base launch file.

The `mode` argument selects the execution context.

Common modes include:

* `live` -> starts the real robot base and teleoperation stack
* `simulation_gazebo` -> starts Gazebo integration when selected by the profiles
* `simulation_gazebo_classic` -> starts Gazebo Classic integration when selected by the profiles

`mobile_base.launch.py` does not start the simulator itself. Simulation worlds and entities are usually started by `romea_simulation_meta_bringup` or by a higher-level robot launch file.

---

## 6) Relation with robot-specific bringup packages

Robot-specific bringup packages must follow the `<robot_name>_bringup` naming convention, for example `cinteo_bringup`, `adap2e_bringup`, `husky_bringup` or `scout_bringup`.

These packages provide the concrete generation and launch functions used by `romea_mobile_base_meta_bringup`:

* mobile base configuration generation
* URDF generation
* ros2_control generation
* base launch file
* teleoperation launch file

This keeps the meta-bringup layer generic while allowing each robot package to provide its own hardware, description and launch specialization.
