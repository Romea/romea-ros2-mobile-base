# romea_mobile_base_description

## 1) Overview

`romea_mobile_base_description` provides reusable description tools for building mobile robot URDF and ros2_control descriptions.

It contains:

* generic URDF/Xacro descriptions for common mobile base architectures
* generic ros2_control Xacro descriptions for these architectures
* a Python module able to parse robot configuration files and extract a simplified configuration used by control, kinematics, teleoperation and bringup packages

This package does not describe one complete robot by itself. Instead, it provides the common description layer used by robot-specific packages such as `cinteo_description`, `adap2e_description`, `husky_description` or `scout_description`.

---

## 2) Robot description

### 2.1 Concept

Robot-specific description packages define their mobile base in a YAML configuration file.

This file contains the concrete parameters of the robot:

* mobile base architecture
* geometry
* inertia
* actuator command limits
* sensor feedback characteristics
* control point
* URDF joint names
* URDF link names

The global structure of a robot configuration file is:

```yaml
type: <mobile_base_architecture>

geometry:
  ...

<steering_control_section>:
  command:
    ...
  sensor:
    ...

<speed_control_section>:
  command:
    ...
  sensor:
    ...

inertia:
  ...

control_point: [...]

joints:
  ...

links:
  ...
```

`romea_mobile_base_description` provides the common tools used to interpret this configuration and to build reusable URDF, ros2_control and compact runtime configuration artifacts.

---

### 2.2 Format

The following sections describe how each part of the configuration is filled and how it is interpreted by the Python module.

---

#### type

The `type` field defines the mobile base architecture.

Supported values include:

| Type | Description | Kinematic family | Command type |
|:----:|-------------|:-----------------|:-------------|
| `1FAS2FWD` | one front-axle steering, two front-wheel drive | `one_axle_steering` | `one_axle_steering` |
| `1FAS2RWD` | one front-axle steering, two rear-wheel drive | `one_axle_steering` | `one_axle_steering` |
| `1FAS4WD` | one front-axle steering, four-wheel drive | `one_axle_steering` | `one_axle_steering` |
| `2AS2FWD` | two-axle steering, two front-wheel drive | `two_axle_steering` | `two_axle_steering` |
| `2AS2RWD` | two-axle steering, two rear-wheel drive | `two_axle_steering` | `two_axle_steering` |
| `2AS4WD` | two-axle steering, four-wheel drive | `two_axle_steering` | `two_axle_steering` |
| `2FWS2FWD` | two front-wheel steering, two front-wheel drive | `two_wheel_steering` | `one_axle_steering` |
| `2FWS2RWD` | two front-wheel steering, two rear-wheel drive | `two_wheel_steering` | `one_axle_steering` |
| `2FWS4WD` | two front-wheel steering, four-wheel drive | `two_wheel_steering` | `one_axle_steering` |
| `2TD` | two continuous-track drive | `skid_steering` | `skid_steering` |
| `2THD` | two high-drive continuous-track drive | `skid_steering` | `skid_steering` |
| `2TTD` | two tank-style continuous-track drive | `skid_steering` | `skid_steering` |
| `4WD` | four-wheel drive | `skid_steering` | `skid_steering` |
| `4WS4WD` | four-wheel steering, four-wheel drive | `four_wheel_steering` | `two_axle_steering` |

The kinematic family describes the physical architecture of the mobile base. The command type describes the command interface used by controllers and teleoperation nodes. Some physical architectures share a command type with another family; for example, `2FWS*` bases are commanded as `one_axle_steering`, while `4WS4WD` bases are commanded as `two_axle_steering`.

---

#### geometry

The `geometry` section describes the physical layout of the mobile base.

It commonly contains:

* `axles_distance` or `fake_wheelbase` -> distance used as wheelbase
* `front_axle.wheels_distance` -> distance between front left and front right wheels
* `rear_axle.wheels_distance` -> distance between rear left and rear right wheels
* `wheels_distance` -> distance between left and right wheels for simple wheel-drive bases
* `tracks_distance` -> distance between left and right continuous tracks for continuous-track bases
* `wheels.radius` and `wheels.width` -> wheel dimensions
* `aabb` -> approximate bounding box used by URDF and visualization tools
* `ground_clearance` -> distance between ground and lowest robot structure

The Python module extracts `wheelbase` and `track` from this section.

Example:

```yaml
geometry:
  axles_distance: 1.55
  front_axle:
    wheels_distance: 1.02
    wheels:
      radius: 0.20
      width: 0.09
      x_offset: -0.04
      hub_carrier_offset: 0.07
  rear_axle:
    wheels_distance: 1.13
    wheels:
      radius: 0.285
      width: 0.112
      hub_carrier_offset: 0.0
  aabb:
    length: 2.11
    width: 1.00
    height: 0.25
    center: [0.0, 0.0, 0.78]
  ground_clearance: 0.65
```

---

#### control sections

Control sections describe the actuators and feedback available on the robot.

Depending on the architecture, the configuration may contain:

* `wheels_speed_control`
* `front_wheels_speed_control`
* `rear_wheels_speed_control`
* `tracks_speed_control`
* `front_axle_steering_control`
* `rear_axle_steering_control`
* `axles_steering_control`
* `front_wheels_steering_control`
* `rear_wheels_steering_control`
* `wheels_steering_control`

Each control section usually contains:

```yaml
command:
  maximal_speed: 1.0
  maximal_acceleration: 1.0

sensor:
  speed_std: 0.1
  speed_range: 1.0
```

or, for steering:

```yaml
command:
  maximal_angle: 0.576
  maximal_angular_speed: 0.243

sensor:
  angle_std: 0.0087
  angle_range: 0.576
```

The Python module uses these fields to compute command limits, such as maximal linear speed, maximal angular speed or maximal steering angle.

---

#### inertia

The `inertia` section describes the main inertial properties of the mobile base:

```yaml
inertia:
  mass: 800.0
  center: [-0.1, 0.0, 0.2]
  z_moment: 0.0
```

These values are used by generated configuration files and can also be used by controllers, simulation and analysis tools.

---

#### control_point

The `control_point` field defines the point where the mobile base command or kinematic model is expressed:

```yaml
control_point: [-0.75, 0.0, 0.0]
```

It is expressed in the base frame and is typically used by control and localisation components.

---

#### joints and links

The `joints` and `links` sections define the names used in generated URDF and ros2_control descriptions.

They must be consistent with the selected architecture and with the Xacro macros used by the robot-specific description package.

Example:

```yaml
joints:
  base_footprint_joint_name: base_footprint_joint
  inertial_joint_name: inertial_joint
  front_axle_steering_joint_name: front_axle_steering_joint
  rear_left_wheel_spinning_joint_name: rear_left_wheel_spinning_joint
  rear_right_wheel_spinning_joint_name: rear_right_wheel_spinning_joint

links:
  base_link_name: base_link
  base_footprint_link_name: base_footprint
  inertial_link_name: inertial_link
  front_axle_steering_link_name: front_axle_steering_link
  rear_left_wheel_spinning_link_name: rear_left_wheel_spinning_link
  rear_right_wheel_spinning_link_name: rear_right_wheel_spinning_link
```

---

### 2.3 Example

The following example describes a one-front-axle steering mobile base with rear wheel drive.

```yaml
# kinematic family: one_axle_steering
type: 1FAS2RWD

geometry:
  axles_distance: 1.55
  front_axle:
    wheels_distance: 1.02
    wheels:
      radius: 0.20
      width: 0.09
      x_offset: -0.04
      hub_carrier_offset: 0.07
  rear_axle:
    wheels_distance: 1.13
    wheels:
      radius: 0.285
      width: 0.112
      hub_carrier_offset: 0.0
  aabb:
    length: 2.11
    width: 1.00
    height: 0.25
    center: [0.0, 0.0, 0.78]
  ground_clearance: 0.65

front_axle_steering_control:
  command:
    maximal_angle: 0.576
    maximal_angular_speed: 0.243
  sensor:
    angle_std: 0.0087
    angle_range: 0.576

rear_wheels_speed_control:
  command:
    maximal_speed: 1.0
    maximal_acceleration: 1.0
  sensor:
    speed_std: 0.1
    speed_range: 1.0

inertia:
  mass: 800.0
  center: [-0.1, 0.0, 0.2]
  z_moment: 0.0

control_point: [-0.75, 0.0, 0.0]

joints:
  base_footprint_joint_name: base_footprint_joint
  inertial_joint_name: inertial_joint
  front_axle_steering_joint_name: front_axle_steering_joint
  front_left_wheel_steering_joint_name: front_left_wheel_steering_joint
  front_right_wheel_steering_joint_name: front_right_wheel_steering_joint
  front_left_wheel_spinning_joint_name: front_left_wheel_spinning_joint
  front_right_wheel_spinning_joint_name: front_right_wheel_spinning_joint
  rear_left_wheel_spinning_joint_name: rear_left_wheel_spinning_joint
  rear_right_wheel_spinning_joint_name: rear_right_wheel_spinning_joint

links:
  base_link_name: base_link
  base_footprint_link_name: base_footprint
  inertial_link_name: inertial_link
  front_axle_steering_link_name: front_axle_steering_link
  front_left_wheel_steering_link_name: front_left_wheel_steering_link
  front_right_wheel_steering_link_name: front_right_wheel_steering_link
  front_left_wheel_spinning_link_name: front_left_wheel_spinning_link
  front_right_wheel_spinning_link_name: front_right_wheel_spinning_link
  rear_left_wheel_spinning_link_name: rear_left_wheel_spinning_link
  rear_right_wheel_spinning_link_name: rear_right_wheel_spinning_link
```

---

## 3) URDF and ros2_control descriptions

`romea_mobile_base_description` provides reusable Xacro files used by robot-specific description packages to build complete robot descriptions.

These files are not usually used alone. They are included by `<robot_name>_description` packages, which provide the concrete geometry, inertia, joints, links and robot-specific visual elements.

### 3.1 URDF descriptions

The `urdf/` directory contains generic chassis Xacro descriptions that provide the structural part of supported mobile base architectures: links, joints, inertial elements, collisions and reusable macros for wheels, steering elements and continuous-track layouts.

Chassis templates follow the `base<architecture_family>.chassis.xacro` naming pattern. Some templates are shared by several vehicle types; for example, `base1FASxxx.chassis.xacro` is used by `1FAS2FWD`, `1FAS2RWD` and `1FAS4WD`.

The chassis templates define the link and joint hierarchy of the mobile base in three layers:

* base layer: `base_footprint`, `base_link`, inertial link and fixed joints
* steering layer: axle steering links/joints or wheel steering links/joints, depending on the architecture
* drive layer: spinning wheel links/joints, sprocket wheels, idler wheels and related joints

Shared helper macros factor common pieces used by the chassis templates:

* `common.xacro` defines the base footprint, base link, inertial link, chassis collision box and Gazebo contact properties
* `wheel.xacro` defines wheel links, spinning joints, cylindrical collisions and wheel transmissions
* `wheel_steered.xacro` combines a steering link/joint with a spinning wheel
* `axle_steering.xacro` defines axle steering links, joints and transmissions

These URDF descriptions also define collision boxes, wheel collision cylinders and insertion points where robot-specific visual meshes can be attached. Robot-specific description packages include these Xacro files and pass their own configuration values to generate a complete robot URDF.

---

### 3.2 ros2_control descriptions

The `ros2_control/` directory contains generic ros2_control Xacro descriptions that provide the control interface part of supported mobile base architectures: hardware plugins, hardware parameters, joint command interfaces and joint state interfaces.

ros2_control templates follow the `base<vehicle_type>.ros2_control.xacro` naming pattern, where `<vehicle_type>` corresponds to the `type` field of the robot configuration.

The ros2_control templates define the controller-manager interface in three layers:

* hardware layer: live or simulation hardware plugin selection, depending on the selected mode
* parameter layer: geometry and control parameters such as wheelbase, track, wheel radius and hub carrier offsets
* joint interface layer: command and state interfaces for steering, wheel spinning or continuous-track joints

These files declare velocity command interfaces for driven wheels or continuous tracks, position command interfaces for steering joints, state interfaces for all controlled joints and command limits derived from wheel radius, steering limits and maximal speeds.

Robot-specific description packages specialize these descriptions by selecting the appropriate architecture and providing robot-specific joint names, geometry values and hardware plugin configuration.

---

## 4) Python API

The package provides the `romea_mobile_base_description` Python module to parse robot configuration files and derive useful configuration values.

---

### 4.1 Configuration helpers

The Python module also exposes helper functions used to extract common values from a full robot configuration.

| Function | Description |
|:---------|-------------|
| `get_kinematic_type(base_description)` | Returns the kinematic family associated with the mobile base architecture. |
| `get_command_type(base_description)` | Returns the command family used by controllers and teleoperation nodes. |
| `get_wheelbase(base_description)` | Returns the distance between axles from `geometry.axles_distance` or `geometry.fake_wheelbase`. |
| `get_wheelbase_or(base_description, default_value)` | Returns the wheelbase when available, otherwise returns the provided default value. |
| `get_track(base_description)` | Returns the distance between left and right wheels or continuous tracks, according to the selected architecture and control sections. |
| `get_inertia(base_description)` | Returns the `inertia` section of the robot configuration. |
| `get_command_limits(base_description)` | Returns normalized command limits according to the kinematic family. |

These helpers are used internally by `get_complete_configuration()`, but they can also be used by robot-specific description or bringup packages when they need only one part of the parsed configuration.

---

### 4.2 get_complete_configuration

`get_complete_configuration(base_description)` converts a full robot configuration into a compact configuration shared by control, kinematics, teleoperation and bringup packages.

It extracts and normalizes:

* `type` -> mobile base architecture
* `command_type` -> command family used by controllers and teleoperation
* `command_limits` -> linear speed, angular speed or steering limits
* `inertia` -> mass, center and z moment
* `wheelbase` -> distance between axles
* `track` -> distance between left and right wheels or continuous tracks

Example:

```python
from romea_mobile_base_description import get_complete_configuration

configuration = get_complete_configuration(base_description)
```

Generated compact configuration:

```yaml
type: 1FAS2RWD
command_type: one_axle_steering
command_limits:
  minimal_longitudinal_speed: -1.0
  maximal_longitudinal_speed: 1.0
  maximal_steering_angle: 0.576
inertia:
  mass: 800.0
  center: [-0.1, 0.0, 0.2]
  z_moment: 0.0
wheelbase: 1.55
track: 1.02
```

Units associated with this compact configuration are defined in `config/specifications_units.yaml`.

---

## 5) Relation with robot-specific description packages

`romea_mobile_base_description` provides the generic description layer for mobile robot bases.

Robot-specific description packages must follow the `<robot_name>_description` naming convention, for example `cinteo_description`, `adap2e_description`, `husky_description` or `scout_description`.

These packages extend the generic layer by combining reusable files from `romea_mobile_base_description` with concrete platform data:

* geometry and inertia
* wheel, continuous track or steering layout
* link and joint names
* robot-specific meshes and visual elements
* robot-specific URDF/Xacro files
* robot-specific ros2_control specialization

In this architecture, `romea_mobile_base_description` provides reusable building blocks, while robot-specific description packages instantiate and specialize them for real robots. This produces the final URDF and ros2_control descriptions consumed by `robot_state_publisher`, `controller_manager`, simulation packages and meta-bringup tools.

Robot-specific description packages can then be selected by `romea_mobile_base_meta_bringup` when a mobile base meta-description is used to instantiate a complete robot system.
