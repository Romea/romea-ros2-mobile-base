# romea_mobile_base_controllers

## 1) Overview

`romea_mobile_base_controllers` provides `ros2_control` controller plugins for ROMEA mobile bases.

The controllers receive high-level mobile base commands, clamp them according to the configured command limits, convert them into joint commands with the kinematic models from `romea_core_mobile_base`, and publish odometry feedback computed from the joint states.

This package is the control layer between:

* the mobile base description, which provides the robot geometry, joints and command limits,
* the mobile base hardware, which exposes the joints through `ros2_control`,
* the teleoperation or autonomy nodes, which publish mobile base command messages.

## 2) Controller concept

Each controller is built from two parts:

* a generic `MobileBaseController`, which implements the `ros2_control` lifecycle, command subscription, command timeout, command clamping, state publication and dead reckoning;
* an architecture-specific controller interface, which maps a mobile base command or odometry frame to the command and state interfaces of the corresponding joints.

At runtime, the controller:

1. reads the configured `base_info` parameters, usually derived from the robot configuration files stored in the `config/` directory of the `<robot_name>_description` package;
2. requests the command and state interfaces required by the selected mobile base architecture;
3. subscribes to the command topic associated with the command type;
4. writes velocity or position commands to the joint interfaces;
5. reads joint states and publishes odometry, kinematic feedback and optionally the `odom -> base_link` TF.

## 3) Controller status

The table below summarizes the controller roadmap for the supported mobile base architectures. Several controllers are already exported as `ros2_control` plugins, while others are still planned.

Standard controller names follow the `MobileBaseController<mobile_base_architecture>` convention, for example `MobileBaseController4WD` for `4WD` robots such as EffibotE3, or `MobileBaseController4WS4WD` for `4WS4WD` robots such as Adap2e. Mobile base architectures are described in the [`romea_mobile_base_description` README](../romea_mobile_base_description/README.md). Plugin names in the table are relative to the `romea_mobile_base_controllers/` prefix.

The real robot validation status indicates whether the controller has already been used on an actual robot. Controllers validated on real robots are considered operational.

| Controller | Status | Real robot validation | Used by |
|---|---|---|---|
| `MobileBaseController2WD` | exported plugin | no | - |
| `MobileBaseController4WD` | exported plugin | yes | EffibotE3, Husky, Scout, Campero rubber |
| `MobileBaseController4MWD` | exported plugin, uses the `4WD` joint interface with mecanum kinematics | no | - |
| `MobileBaseController2TD` | exported plugin | yes | Ceol |
| `MobileBaseController1FAS2RWD` | exported plugin | yes | Cinteo, Hunter |
| `MobileBaseController1FAS2FWD` | exported plugin | no | - |
| `MobileBaseController1FAS4WD` | planned, not implemented | no | - |
| `MobileBaseController2AS2FWD` | planned, not implemented | no | - |
| `MobileBaseController2AS2RWD` | planned, not implemented | no | - |
| `MobileBaseController2AS4WD` | exported plugin | yes | Aroco, Robucar |
| `MobileBaseController2FWS2FWD` | exported plugin | no | - |
| `MobileBaseController2FWS2RWD` | exported plugin | yes | Pom basic |
| `MobileBaseController2FWS4WD` | exported plugin | yes | Pom 4x4 |
| `MobileBaseController4WS4WD` | exported plugin | yes | Adap2e |
| `MobileBaseEnhancedController2TD` | exported plugin, adds IMU angular speed feedback | yes | Ceol |
| `MobileBaseEnhancedController4WD` | planned, not implemented | no | - |

## 4) ROS 2 interfaces

### 4.1 Subscribed topics

The command topic depends on the command family selected by the controller plugin. Message names in this table are relative to the `romea_mobile_base_msgs/msg/` prefix.

| Topic | Message | Used by |
|---|---|---|
| `controller/cmd_skid_steering` | `SkidSteeringCommand` | skid steering and continuous track controllers |
| `controller/cmd_one_axle_steering` | `OneAxleSteeringCommand` | one axle steering and two front wheel steering controllers |
| `controller/cmd_two_axle_steering` | `TwoAxleSteeringCommand` | two axle steering and four wheel steering controllers |
| `controller/cmd_omni_steering` | `OmniSteeringCommand` | mecanum/omni steering controllers |

`MobileBaseEnhancedController2TD` also subscribes to `imu/data` (`sensor_msgs/msg/Imu`) to regulate the angular speed with IMU feedback.

### 4.2 Published topics

All mobile base controllers publish:

| Topic | Message | Description |
|---|---|---|
| `controller/odom` | `nav_msgs/msg/Odometry` | dead reckoning pose and twist in the odometry frame |
| `controller/kinematic` | `romea_mobile_base_msgs/msg/KinematicMeasureStamped` | compact kinematic feedback: longitudinal speed, lateral speed when available, angular speed and curvature |
| `controller/odometry` | architecture-dependent measure message | feedback expressed in the same operating space as the command family |
| `/tf` | `tf2_msgs/msg/TFMessage` | optional `odom -> base_link` transform when `controller.enable_odom_tf` is enabled |

## 5) Parameters

### 5.1 Common parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `controller.base_frame_id` | string | `base_link` | Frame attached to the mobile base. |
| `controller.odom_frame_id` | string | `odom` | Odometry frame used by dead reckoning. |
| `controller.enable_odom_tf` | bool | `false` | Enables publication of the `odom -> base_link` TF. |
| `controller.publish_rate` | double | `50.0` | Publication rate of `controller/odom`, `controller/kinematic` and `controller/odometry`. |
| `controller.timeout` | double | `0.5` | Delay, in seconds, after which the controller sends a null command if no new command is received. |
| `controller.joints_prefix` | string | empty | Prefix added to all joint names read from `base_info.joints`. |

### 5.2 Mobile base information

`base_info` is a dictionary parameter whose main entries used by the controllers are:

| Entry | Description |
|---|---|
| `base_info.geometry` | geometric dimensions required by the kinematic model, such as track, wheelbase, wheel radius or continuous-track dimensions depending on the architecture |
| `base_info.<steering_control_section>` | steering command and sensor information when the architecture has steering joints |
| `base_info.<speed_control_section>` | wheel or track speed command and sensor information |
| `base_info.inertia` | mass, center of mass and yaw inertia used by higher-level control components |
| `base_info.control_point` | point of the mobile base where commands and kinematic feedback are expressed |
| `base_info.joints` | names of the steering, spinning or sprocket joints consumed by the controller |

In normal use, `base_info` is obtained by loading the robot configuration file provided by the corresponding `<robot_name>_description` package, then passing this configuration to the controller as ROS parameters. Its structure is described by `romea_mobile_base_description` and read by the helpers from `romea_mobile_base_utils`.

### 5.3 Command limits

Command limits are stored under `controller.command_limits`. The expected fields depend on the command family:

| Command family | Parameters |
|---|---|
| Skid steering | `minimal_longitudinal_speed`, `maximal_longitudinal_speed`, `maximal_angular_speed` |
| Omni steering | `minimal_longitudinal_speed`, `maximal_longitudinal_speed`, `maximal_lateral_speed`, `maximal_angular_speed` |
| One axle steering | `minimal_longitudinal_speed`, `maximal_longitudinal_speed`, `maximal_steering_angle` |
| Two axle steering | `minimal_longitudinal_speed`, `maximal_longitudinal_speed`, `maximal_front_steering_angle`, `maximal_rear_steering_angle` |

### 5.4 Enhanced controller parameters

`MobileBaseEnhancedController2TD` adds angular speed feedback parameters:

| Parameter | Description |
|---|---|
| `controller.angular_speed.pid` | PID gains used to regulate the angular speed command from IMU feedback. |
| `controller.angular_speed.filter.alpha` | Low-pass filter coefficient applied to the IMU angular speed. |

## 6) Example configuration

The controller configuration is usually generated by the mobile base meta-bringup. A minimal handwritten example for a `4WD` skid steering controller is split between the controller manager configuration and the controller runtime parameters.

The controller manager configuration declares the plugin type:

```yaml
/**:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    mobile_base_controller:
      type: romea_mobile_base_controllers/MobileBaseController4WD
```

The controller node itself receives the runtime parameters:

```yaml
/**:
  ros__parameters:
    update_rate: 10

    controller:
      base_frame_id: base_link
      odom_frame_id: odom
      enable_odom_tf: true
      publish_rate: 50.0
      timeout: 0.5
      joints_prefix: ""
      command_limits:
        minimal_longitudinal_speed: -2.0
        maximal_longitudinal_speed: 2.0
        maximal_angular_speed: 1.5

    base_info:
      geometry:
        axles_distance: 1.20
        front_axle: &axle
          wheels_distance: 0.80
          wheels: {radius: 0.25, width: 0.10, hub_carrier_offset: 0.0}
        rear_axle: *axle
      wheels_speed_control:
        command: {maximal_speed: 10.0, maximal_acceleration: 5.0}
        sensor: {speed_std: 0.05, speed_range: 20.0}
      inertia:
        mass: 250.0
        center: [0.0, 0.0, 0.35]
        z_moment: 50.0
      control_point: [0.0, 0.0, 0.0]
      joints:
        front_left_wheel_spinning_joint_name: front_left_wheel_spinning_joint
        front_right_wheel_spinning_joint_name: front_right_wheel_spinning_joint
        rear_left_wheel_spinning_joint_name: rear_left_wheel_spinning_joint
        rear_right_wheel_spinning_joint_name: rear_right_wheel_spinning_joint
```

## 7) Launch helpers

The package provides `mobile_base_controller.launch.py`, which loads a mobile base configuration and a controller parameter configuration, combines them into the controller ROS parameters, applies the optional `joints_prefix`, and spawns both the selected mobile base controller and `joint_state_broadcaster`.

The usual launch arguments for `mobile_base_controller.launch.py` are:

| Argument | Description |
|---|---|
| `controller_manager_name` | Name of the `controller_manager` node. |
| `controller_name` | Name of the controller instance to spawn. |
| `base_configuration_file_path` | YAML file containing the compact `base_info` configuration. |
| `base_controller_configuration_file_path` | YAML file containing the common controller parameters. |
| `joints_prefix` | Optional prefix added to all joints used by the controller. |

The controller plugin type itself is declared in the controller manager configuration, under the selected `controller_name`.

## 8) Relation with other mobile base packages

`romea_mobile_base_controllers` does not define the robot model by itself. It consumes the mobile base configuration and joint names produced from `romea_mobile_base_description` and assembled by `romea_mobile_base_meta_bringup`.

The hardware package exposes the joint command and state interfaces requested by these controllers, while teleoperation or autonomy packages publish the command messages consumed on the `controller/cmd_*` topics.
