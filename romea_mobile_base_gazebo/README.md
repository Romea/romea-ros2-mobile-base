# romea_mobile_base_gazebo

## 1) Overview

`romea_mobile_base_gazebo` provides `ros2_control` hardware plugins for mobile base simulation with Gazebo, through `gz_ros2_control`.

The package connects ROMEA mobile base controllers to Gazebo simulated joints. It is built on top of `romea_mobile_base_simulation`, which provides the conversion layer between:

* the controller-side mobile base architecture, consumed by `romea_mobile_base_controllers`;
* the simulation-side joint layout, exposed by the Gazebo model.

This is a simulator-specific integration: the hardware plugin is loaded in the Gazebo simulation process through `gz_ros2_control`, so the simulator and the `controller_manager` exchange data directly through Gazebo joint entities.

## 2) Architecture

Each Gazebo system plugin combines two layers:

* a `GazeboInterface*`, which reads and writes Gazebo joints;
* a `SimulationInterface*`, provided by `romea_mobile_base_simulation`, which converts between controller-side joints and simulation-side joints.

At runtime, the plugin:

1. receives the `ros2_control` hardware information generated from the robot description;
2. creates the Gazebo joint interface matching the simulated model;
3. creates the simulation interface matching the controller-side mobile base architecture;
4. exports the command and state interfaces expected by the mobile base controller;
5. forwards controller commands to Gazebo joints;
6. converts Gazebo joint states back into controller-side feedback.

From the controller point of view, this behaves like a normal mobile base hardware interface: controllers write actuator commands to `ros2_control` command interfaces and read state interfaces to compute odometry. The Gazebo system plugin handles the connection between these interfaces and the simulated joints.

## 3) Exported plugins

Plugin names follow the `romea_mobile_base_gazebo/GazeboSystemInterface<mobile_base_architecture>` convention. Plugin names in the table are relative to the `romea_mobile_base_gazebo/` prefix.

| Plugin | Controller-side architecture | Simulation-side architecture | Used by |
|---|---|---|---|
| `GazeboSystemInterface4WD` | `4WD` | `4WD` | EffibotE3, Husky, Scout, Campero rubber |
| `GazeboSystemInterface4WS4WD` | `4WS4WD` | `4WS4WD` | Adap2e |
| `GazeboSystemInterface1FAS2RWD` | `1FAS2RWD` | `1FAS4WD` | Cinteo, Hunter |
| `GazeboSystemInterface2AS4WD` | `2AS4WD` | `2AS4WD` | Aroco, Robucar |
| `GazeboSystemInterface2FWS4WD` | `2FWS4WD` | `2FWS4WD` | Pom 4x4 |
| `GazeboSystemInterface2FWS2RWD` | `2FWS2RWD` | `2FWS4WD` | Pom basic |
| `GazeboSystemInterface2THD` | `2TD` | `2THD` | Ceol |

Other Gazebo interface classes are present in the library and tests, but are not currently exported as Gazebo system plugins because no robot package uses or validates them yet.

## 4) Relation with other mobile base packages

`romea_mobile_base_gazebo` is the Gazebo-specific implementation of the simulation architecture:

* `romea_mobile_base_description` selects the Gazebo system plugin in the `ros2_control` description;
* `romea_mobile_base_controllers` commands the controller-side hardware interfaces;
* `romea_mobile_base_simulation` provides the architecture conversion interfaces;
* `romea_mobile_base_gazebo` connects those interfaces to Gazebo joints through `gz_ros2_control`.
