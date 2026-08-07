# romea_ros2_mobile_base

## Overview

`romea_ros2_mobile_base` groups the ROS2 packages used to describe, launch, command, control and simulate mobile bases in the ROMEA ecosystem.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `romea_mobile_base` | Metapackage that groups the mobile base ROMEA ROS2 packages. |
| `romea_mobile_base_description` | Shared description layer for mobile base configurations, URDF generation and ros2_control descriptions. |
| `romea_mobile_base_meta_bringup` | Main integration entry point for generating mobile base configuration files, URDF descriptions and launch files from a meta-description. |
| `romea_mobile_base_msgs` | ROS2 messages used to exchange mobile base commands and motion measurements. |
| `romea_mobile_base_utils` | Shared utilities for mobile base parameters, command types, measures, conversions and topic naming. |
| `romea_mobile_base_controllers` | `ros2_control` controller plugins for the supported mobile base architectures. |
| `romea_mobile_base_hardware` | Reusable `ros2_control` hardware abstractions used by robot-specific hardware packages. |
| `romea_mobile_base_teleop` | Joystick teleoperation nodes and configuration helpers for mobile base command families. |
| `romea_mobile_base_simulation` | Generic simulation adaptation layer between controller-side mobile base architectures and simulator-side joint layouts. |
| `romea_mobile_base_gazebo` | Gazebo integration for mobile base simulation through `gz_ros2_control`. |
| `romea_mobile_base_gazebo_classic` | Gazebo Classic integration for mobile base simulation through `gazebo_ros2_control`. |

## Usage

In most cases, start with `romea_mobile_base_meta_bringup`. It is the user-facing entry point of the stack. A mobile base meta-description selects the concrete robot model and delegates platform-specific configuration, URDF generation and launch generation to the corresponding `<robot_name>_bringup` package.

The mobile base stack does not describe one complete robot by itself. It provides the common description, controlllers, hardware, simulation, teleoperation and message layers shared by robot-specific packages.

Robot-specific packages extend this stack through their own description, hardware and bringup packages. For example, a `<robot_name>_description` package provides the concrete geometry and ros2_control description, a `<robot_name>_hardware` package provides the real robot communication layer, and a `<robot_name>_bringup` package connects them to the generic `romea_mobile_base_meta_bringup` workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `romea_ros2_mobile_base` project was developed by Jean Laneurit in the context of ROMEA projects involving the TSCF research unit.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
