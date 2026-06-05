# romea_mobile_base_msgs

## 1) Overview

`romea_mobile_base_msgs` defines the ROS 2 messages used to exchange mobile base commands and mobile base motion measurements in the ROMEA stack.

This package only provides message definitions. It does not contain controllers, hardware interfaces, teleoperation nodes or launch files.

The messages are used by packages such as:

* `romea_mobile_base_teleop`, which publishes mobile base command messages from joystick or keyboard inputs;
* `romea_mobile_base_controllers`, which consumes command messages and publishes measured mobile base motion;
* higher-level robot packages, which remap or expose these topics as part of a complete robot bringup.

## 2) Message families

The package provides three kinds of messages:

| Family | Role |
|---|---|
| `*Command` | desired mobile base motion sent to a controller |
| `*Measure` | measured or estimated mobile base motion, with covariance |
| `*MeasureStamped` | measured or estimated mobile base motion with a ROS header |

The command and measure messages are specialized according to the kinematic command type used by the mobile base.

## 3) Supported command and measure types

| Command type | Command message | Measure message | Main fields |
|---|---|---|---|
| Skid steering | `SkidSteeringCommand` | `SkidSteeringMeasure` | longitudinal speed, angular speed |
| One axle steering | `OneAxleSteeringCommand` | `OneAxleSteeringMeasure` | longitudinal speed, steering angle |
| Two axle steering | `TwoAxleSteeringCommand` | `TwoAxleSteeringMeasure` | longitudinal speed, front and rear steering angles |
| Omni steering | `OmniSteeringCommand` | `OmniSteeringMeasure` | longitudinal speed, lateral speed, angular speed |

Each measure message also contains a covariance array adapted to the size of the corresponding motion vector.

Stamped variants are available for each measure type:

| Measure message | Stamped message |
|---|---|
| `SkidSteeringMeasure` | `SkidSteeringMeasureStamped` |
| `OneAxleSteeringMeasure` | `OneAxleSteeringMeasureStamped` |
| `TwoAxleSteeringMeasure` | `TwoAxleSteeringMeasureStamped` |
| `OmniSteeringMeasure` | `OmniSteeringMeasureStamped` |

## 4) Kinematic measure

`KinematicMeasure` provides a compact representation of the mobile base motion after conversion to a common kinematic form.

It contains:

| Field | Description |
|---|---|
| `longitudinal_speed` | forward speed of the mobile base |
| `lateral_speed` | lateral speed of the mobile base |
| `angular_speed` | yaw rate of the mobile base |
| `instantaneous_curvature` | instantaneous trajectory curvature |
| `covariance` | covariance associated with the kinematic measure |

`KinematicMeasureStamped` adds a `std_msgs/Header` to this common kinematic measure.

## 5) Relation with other mobile base packages

`romea_mobile_base_msgs` defines the message interface shared by the mobile base packages:

* `romea_mobile_base_description` defines the robot configuration and the mobile base architecture;
* `romea_mobile_base_teleop` selects the command message type according to the teleoperation and mobile base configuration;
* `romea_mobile_base_controllers` consumes the command messages and produces mobile base measures;
* `romea_mobile_base_utils` provides helper code to convert and handle the corresponding C++ data structures.
