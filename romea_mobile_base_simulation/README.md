# romea_mobile_base_simulation

## 1) Overview

`romea_mobile_base_simulation` provides the adaptation layer used to connect ROMEA mobile base controllers to simulated robots, even when the simulated model exposes a different joint layout from the real robot controller.

It bridges controller-side and simulation-side joint layouts by:

* converting the joint commands expected by the real mobile base controller into the joint commands expected by the simulated model;
* converting simulated joint states back into the joint states expected by the controller;
* providing reusable `GenericSimulationSystemInterface` plugins, implemented as `hardware_interface::SystemInterface`, that exchange commands and feedback with a simulator through `sensor_msgs/msg/JointState` messages.

## 2) Simulation concept

In the ROMEA stack, controllers are written as if they were controlling the real robot hardware. A simulator, however, may expose a more complete or slightly different set of joints. For example, a real `1FAS2RWD` robot controller commands one front axle steering joint and two rear spinning wheels, while the simulated model can expose individual front wheel steering joints and additional wheel joints.

The package separates two notions:

* **controller-side mobile base architecture**: the architecture seen by `romea_mobile_base_controllers`;
* **simulation-side architecture**: the joint layout exposed by the simulator.

For identical layouts, the simulation interface can directly reuse the corresponding hardware interface. For reduced real layouts simulated with a more complete model, a dedicated simulation interface performs the conversion.

The architecture-specific simulation interfaces are reusable building blocks. They can be used directly by simulator-specific packages such as `romea_mobile_base_gazebo` and `romea_mobile_base_gazebo_classic`, or wrapped by a generic ROS 2 hardware plugin.

This gives two integration modes:

* simulator-specific packages use the `SimulationInterface*` classes to implement dedicated integrations where the simulator and the `controller_manager` exchange data through the simulator backend;
* `GenericSimulationSystemInterface*` plugins can be used with remote simulators such as 4DV Virtualiz or Isaac Sim, when a simulator-side adapter can receive joint commands and send joint feedback as `sensor_msgs/msg/JointState` messages.

## 3) Simulation interfaces

The following simulation interfaces are provided by the library:

| Controller-side architecture | Simulation-side architecture | Interface |
|---|---|---|
| `2WD` | `2WD` | `SimulationInterface2WD` |
| `4WD` | `4WD` | `SimulationInterface4WD` |
| `1FAS2FWD` | `1FAS4WD` | `SimulationInterface1FAS2FWD` |
| `1FAS2RWD` | `1FAS4WD` | `SimulationInterface1FAS2RWD` |
| `2AS4WD` | `2AS4WD` | `SimulationInterface2AS4WD` |
| `2FWS2FWD` | `2FWS4WD` | `SimulationInterface2FWS2FWD` |
| `2FWS2RWD` | `2FWS4WD` | `SimulationInterface2FWS2RWD` |
| `2FWS4WD` | `2FWS4WD` | `SimulationInterface2FWS4WD` |
| `4WS4WD` | `4WS4WD` | `SimulationInterface4WS4WD` |
| `2TD` | `2TD` | `SimulationInterface2TD` |
| `2TD` | `2THD` | `SimulationInterface2THD` |
| `2TD` | `2TTD` | `SimulationInterface2TTD` |

The `2TD -> 2THD` and `2TD -> 2TTD` interfaces are used to drive simulated continuous-track models with additional idler or roller joints while keeping a simpler `2TD` controller interface.

## 4) Generic Simulation Interfaces

### 4.1) Concept

`GenericSimulationSystemInterface` wraps an architecture-specific `SimulationInterface*` into a reusable `hardware_interface::SystemInterface` plugin that communicates with a simulator through `sensor_msgs/msg/JointState` topics.

This mode is useful when the simulator is connected through a ROS topic bridge, for example with a remote simulator. The simulator-side bridge is responsible for subscribing to `bridge/<interface_name>/joint_state_command`, applying the commands to the simulated model, and publishing feedback on `bridge/<interface_name>/joint_state_feedback`.

### 4.2) Runtime behavior

At runtime, a `GenericSimulationSystemInterface`:

1. loads the `ros2_control` hardware information generated from the robot description;
2. creates the requested architecture-specific simulation interface;
3. exports the same command and state interfaces as a normal mobile base hardware interface;
4. converts controller-side joint commands into simulation-side joint commands;
5. converts simulation-side joint feedback back into controller-side joint states;
6. exchanges these commands and feedback with the simulator through `sensor_msgs/msg/JointState` topics.

### 4.3) Exported plugins and bridge topics

The package currently exports the following generic `hardware_interface::SystemInterface` plugin:

| Plugin | Simulation interface | Used by |
|---|---|---|
| `romea_mobile_base_simulation/GenericSimulationSystemInterface` | Selected from the `simulation_interfaces` parameters | Remote simulators |

Only this generic simulation system interface is exported. Architecture-specific simulation interfaces are available in the library and are selected from the `ros2_control` parameters.

The generic bridge topics are:

| Topic | Direction | Description |
|---|---|---|
| `bridge/<interface_name>/joint_state_command` | published by this package | joint commands converted for the simulated model |
| `bridge/<interface_name>/joint_state_feedback` | subscribed by this package | simulated joint states returned by the simulator |

Gazebo integration packages use the simulation interfaces more directly, without exposing this generic `JointState` bridge.

## 5) ros2_control integration

In a simulation setup, the `controller_manager` loads the hardware plugin declared in the robot URDF, more precisely in the `<ros2_control>` tag generated by the description package.

This hardware plugin exports:

* command interfaces, where mobile base controllers write actuator commands;
* state interfaces, where mobile base controllers read actuator feedback.

The mobile base controller does not communicate directly with the simulator. It writes commands to the exported command interfaces and reads the exported state interfaces to estimate odometry. The simulation layer is responsible for forwarding these commands to the simulated joints and converting simulator feedback back into the state interfaces expected by the controller.

## 6) Relation with other mobile base packages

`romea_mobile_base_simulation` does not define controllers, robot descriptions or simulator-specific integrations by itself. It sits between them:

* `romea_mobile_base_description` defines the URDF and `ros2_control` descriptions used to select the simulated hardware plugin;
* `romea_mobile_base_controllers` consumes the controller-side command and state interfaces;
* `romea_mobile_base_hardware` provides the base hardware interface classes reused by the simulation layer;
* simulator-specific packages reuse the simulation interfaces directly or connect the generic bridge topics to an external simulator.
