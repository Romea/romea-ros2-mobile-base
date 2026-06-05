# romea_mobile_base_teleop

## 1) Overview

`romea_mobile_base_teleop` provides joystick teleoperation nodes for the mobile base command families used in the ROMEA stack.

It converts `sensor_msgs/msg/Joy` messages into command messages compatible with `romea_mobile_base_controllers` and with common ROS command interfaces.

The package provides:

* teleoperation nodes for skid, omni, one-axle and two-axle steering commands
* default joystick remapping files for common devices
* a launch file that selects the correct teleoperation node from the mobile base configuration
* a Python module that completes and clamps teleoperation configurations from robot and joystick descriptions

Teleoperation is enabled only while `slow_mode` or `turbo_mode` is pressed. When the mode button is released, the node sends a null command once to stop the robot.

If `cmd_output.message_priority` is different from `-1`, the teleoperation output topic is registered in the command multiplexer provided by the `romea_cmd_mux` package. When the priority is `-1`, no command mux registration is attempted.

---

## 2) Teleoperation nodes

All mobile base teleoperation nodes subscribe to:

| Topic | Type | Description |
|-------|------|-------------|
| `joystick/joy` | `sensor_msgs/msg/Joy` | joystick input, usually remapped from the joystick driver topic |

They publish one command topic selected by `cmd_output.message_type`.

### 2.1 Mobile base nodes

| Executable | Command family | Supported output message types | Output topic |
|------------|----------------|--------------------------------|--------------|
| `skid_steering_teleop_node` | skid steering | `geometry_msgs/Twist` | `cmd_vel` |
| `skid_steering_teleop_node` | skid steering | `romea_mobile_base_msgs/SkidSteeringCommand` | `cmd_skid_steering` |
| `omni_steering_teleop_node` | omni steering | `geometry_msgs/Twist` | `cmd_vel` |
| `omni_steering_teleop_node` | omni steering | `romea_mobile_base_msgs/OmniSteeringCommand` | `cmd_omni_steering` |
| `one_axle_steering_teleop_node` | one-axle steering | `geometry_msgs/Twist` | `cmd_vel` |
| `one_axle_steering_teleop_node` | one-axle steering | `ackermann_msgs/AckermannDrive` | `cmd_steer` |
| `one_axle_steering_teleop_node` | one-axle steering | `romea_mobile_base_msgs/OneAxleSteeringCommand` | `cmd_one_axle_steering` |
| `two_axle_steering_teleop_node` | two-axle steering | `four_wheel_steering_msgs/FourWheelSteering` | `cmd_4ws` |
| `two_axle_steering_teleop_node` | two-axle steering | `romea_mobile_base_msgs/TwoAxleSteeringCommand` | `cmd_two_axle_steering` |

The output topic is selected internally by `romea_mobile_base_utils` from the configured message type.

---

## 3) Configuration

Teleoperation nodes use three parameter groups:

| Group | Purpose |
|-------|---------|
| `joystick_mapping` | maps logical teleoperation actions to joystick axis and button ids |
| `cmd_output` | selects the output message type and optional command mux priority |
| `cmd_range` | defines the slow and turbo command limits used by teleoperation |

### 3.1 Common parameters

```yaml
cmd_output:
  message_type: romea_mobile_base_msgs/SkidSteeringCommand
  message_priority: -1

cmd_range:
  maximal_linear_speed:
    slow_mode: 1.0
    turbo_mode: 2.0
```

`message_priority` is optional and defaults to `-1`. With this value, the teleoperation node does not register its output topic in `romea_cmd_mux`.

`turbo_mode` values are optional in user teleoperation configurations generated through the Python helper. Missing turbo values are completed from the mobile base limits.

### 3.2 Joystick mappings

The expected joystick actions depend on the command family.

| Command family | Axes |
|----------------|------|
| `skid_steering` | `linear_speed`, `angular_speed` |
| `omni_steering` | `linear_speed`, `lateral_speed`, `angular_speed` |
| `one_axle_steering` | `linear_speed`, `steering_angle` |
| `two_axle_steering` | `forward_speed`, `backward_speed`, `front_steering_angle`, `rear_steering_angle` |

Mobile base teleoperation nodes use these buttons:

| Button | Purpose |
|--------|---------|
| `slow_mode` | enable teleoperation using slow command limits |
| `turbo_mode` | enable teleoperation using turbo command limits |

Example runtime parameter file for a skid-steering teleoperation node:

```yaml
/**:
  ros__parameters:
    joystick_mapping:
      axes: {linear_speed: 0, angular_speed: 1}
      buttons: {slow_mode: 0, turbo_mode: 1}

    cmd_output:
      message_type: romea_mobile_base_msgs/SkidSteeringCommand
      message_priority: -1

    cmd_range:
      maximal_linear_speed: {slow_mode: 1.0, turbo_mode: 2.0}
      maximal_angular_speed: {slow_mode: 0.5, turbo_mode: 1.0}
```

---

## 4) Default joystick remappings

The `config/` directory contains default remapping files that map symbolic joystick names to the logical teleoperation actions expected by the nodes.

Supported remapping files include:

| Joystick type | Command families |
|---------------|------------------|
| `microsoft_xbox` | `skid_steering`, `omni_steering`, `one_axle_steering`, `two_axle_steering` |
| `sony_dualshock4` | `skid_steering`, `omni_steering`, `one_axle_steering`, `two_axle_steering` |
| `keyboard` | `one_axle_steering`, `two_axle_steering` |

The Python helper selects the default file from:

```text
<joystick_type>_<command_type>_remappings.yaml
```

For example, a Microsoft Xbox joystick used with a skid-steering robot selects:

```text
microsoft_xbox_skid_steering_remappings.yaml
```

---

## 5) Usage

The package provides `teleop.launch.py` to start the teleoperation node matching the mobile base command type.

It reads:

* a compact mobile base configuration, derived from the robot configuration stored in the `config/` directory of the `<robot_name>_description` package using the Python API of `romea_mobile_base_description`
* a joystick configuration, usually selected from the `config/` directory of `romea_joystick_utils` package
* a teleoperation configuration, provided by the user or by a robot-specific bringup package
* the joystick topic to use

Then it:

* determines the mobile base command type from the mobile base configuration
* completes the teleoperation configuration
* clamps requested teleoperation limits to the robot command limits
* applies the joystick remapping
* starts the matching `<command_type>_teleop_node`

Example:

```bash
ros2 launch romea_mobile_base_teleop teleop.launch.py \
  mobile_base_configuration_file_path:=path/to/mobile_base_configuration.yaml \
  joystick_configuration_file_path:=path/to/joystick_configuration.yaml \
  teleop_configuration_file_path:=path/to/teleop_configuration.yaml \
  joystick_topic:=/robot/joystick/joy
```

The launch file remaps the node input topic `joystick/joy` to the provided `joystick_topic`.

---

## 6) Python API

The installed Python module provides helpers to create the runtime configuration passed to the C++ teleoperation nodes.

This configuration is built from:

* `teleop_configuration` -> user teleoperation preferences, output message type and requested command ranges
* `mobile_base_configuration` -> robot command type and physical command limits
* `joystick_configuration` -> joystick type and axis/button layout

Main helpers include:

| Function | Purpose |
|----------|---------|
| `cmd_range_clamp()` | clamps user teleoperation limits against the mobile base command limits |
| `get_default_joystick_remapping()` | loads the default remapping file for a joystick type and command type |
| `get_teleop_complete_configuration()` | combines teleop, mobile base and joystick configurations into the runtime node parameters |

Example:

```python
from romea_mobile_base_teleop import get_teleop_complete_configuration

complete_configuration = get_teleop_complete_configuration(
    teleop_configuration,
    mobile_base_configuration,
    joystick_configuration,
)
```

The resulting configuration contains:

```yaml
cmd_output:
  ...
cmd_range:
  ...
joystick_mapping:
  axes:
    ...
  buttons:
    ...
```

---

## 7) Relation with other mobile base packages

`romea_mobile_base_teleop` is part of the mobile base runtime stack:

* `romea_mobile_base_description` defines the robot command type and command limits
* `romea_mobile_base_utils` provides the command publishers used to publish teleoperation messages
* `romea_joystick_utils` parses `sensor_msgs/msg/Joy` messages from joystick mappings
* `romea_cmd_mux` can arbitrate teleoperation commands with other command sources
* `romea_mobile_base_meta_bringup` uses this package through robot-specific launch files

Together, these packages let a robot-specific bringup select the correct teleoperation node and keep joystick layouts, command limits and output message types consistent with the mobile base architecture.
