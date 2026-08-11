# toio_gazebo

## Introduction

This is a ROS 2 Package to develop package of [toio](https://toio.io/) using Gazebo.

https://github.com/user-attachments/assets/63909751-a5ec-49ca-8c24-c2067247fe8e

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

## Build

```bash
sudo apt update
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/atinfinity/toio_description.git
git clone https://github.com/atinfinity/toio_gazebo.git
cd ..
rosdep install -y -i --from-paths src
colcon build --symlink-install
source ~/dev_ws/install/setup.bash
```

## Launch Gazebo

```bash
ros2 launch toio_gazebo simulation.launch.py world:=<WORLD_SDF_FILEPATH> world_frame:=<WOLRD_FRAME_NAME>
```

An example of command is as follows:

```bash
ros2 launch toio_gazebo simulation.launch.py world:=$HOME/dev_ws/src/toio_gazebo/worlds/toio_a4_map.sdf world_frame:=toio_a4_map
```

## Topics

These match the interface of the real cube provided by
[toio_ros2](https://github.com/atinfinity/toio_ros2), so the same commands work
against the simulation and against the hardware.

### Subscribed topics

| topic | type | description |
| --- | --- | --- |
| `/cmd_vel` | [geometry_msgs/msg/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html) | Desired velocity of the cube |
| `/toio/led` | [std_msgs/msg/ColorRGBA](https://docs.ros2.org/latest/api/std_msgs/msg/ColorRGBA.html) | Color of the indicator LED. `r`, `g` and `b` are taken as 0.0-1.0 and are clipped into that range, `a` is unused. An all zero color turns the LED off |
| `/toio/sound` | [std_msgs/msg/UInt8](https://docs.ros2.org/latest/api/std_msgs/msg/UInt8.html) | Sound effect id, 0-10. Ids outside that range are logged as a warning and ignored |
| `/toio/led_timed` | [toio_msgs/msg/Led](https://github.com/atinfinity/toio_msgs) | Indicator color with the lighting time in the message, for when `led_duration_ms` is not the same for every command |
| `/toio/led_pattern` | [toio_msgs/msg/LedPattern](https://github.com/atinfinity/toio_msgs) | Blink sequence, up to 29 steps. `repeat` 0 repeats until the next indicator command |
| `/toio/melody` | [toio_msgs/msg/Melody](https://github.com/atinfinity/toio_msgs) | MIDI melody, up to 59 notes. Shares the sound throttle with `/toio/sound` |

### Published topics

| topic | type | description |
| --- | --- | --- |
| `/joint_states` | [sensor_msgs/msg/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html) | State of the wheel joints |
| `/tf` | [tf2_msgs/msg/TFMessage](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html) | Pose of the cube, as `map -> center` |

### Launch arguments

> The cube plays a pattern or a melody in its own firmware, so on hardware the
> timing does not depend on the host and a BLE dropout does not interrupt it.
> Nothing in Gazebo can sequence one on its own, so `toio_led_node` keeps the
> time on the ROS side instead: a busy or paused simulation stretches a
> pattern. For the same reason `repeat` 0 loops the LED pattern (a timer can be
> cancelled by the next command) but plays a melody once (a clip already handed
> to the player cannot be stopped).

| argument | default | description |
| --- | --- | --- |
| `led_duration_ms` | `0` | Lighting time of `/toio/led`. `0` keeps the indicator lit until the next command, `10`-`2550` turns it off once the time has elapsed |
| `led_light_intensity` | `1.0` | Intensity of the light of the lamp while it is lit. `0` leaves the lamp lighting up only itself, without casting color on the mat around the cube |
| `sound_volume` | `255` | Volume of `/toio/sound`. Per the toio specification this is mute or full volume only: `0` is mute and every other value is the maximum volume |

On the real cube these are parameters of the toio_ros2 node. Here they are
launch arguments, because the LED is driven by a Gazebo plugin and the sound is
played by a separate node.

### Examples

```bash
# Light the indicator in red, then turn it off
ros2 topic pub --once /toio/led std_msgs/msg/ColorRGBA '{r: 1.0, g: 0.0, b: 0.0, a: 1.0}'
ros2 topic pub --once /toio/led std_msgs/msg/ColorRGBA '{r: 0.0, g: 0.0, b: 0.0, a: 1.0}'
```

Light red for 1.5 seconds, blink blue three times, and play three notes:

```bash
ros2 topic pub --once /toio/led_timed toio_msgs/msg/Led '{color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}, duration_ms: 1500}'
ros2 topic pub --once /toio/led_pattern toio_msgs/msg/LedPattern '{steps: [{color: {r: 0.0, g: 0.0, b: 1.0, a: 1.0}, duration_ms: 400}, {color: {r: 0.0, g: 0.0, b: 0.0, a: 1.0}, duration_ms: 400}], repeat: 3}'
ros2 topic pub --once /toio/melody toio_msgs/msg/Melody '{notes: [{duration_ms: 400, note: 60, volume: 255}, {duration_ms: 400, note: 64, volume: 255}, {duration_ms: 400, note: 67, volume: 255}], repeat: 1}'

# Play a sound effect
ros2 topic pub --once /toio/sound std_msgs/msg/UInt8 '{data: 6}'

# Let the cube turn the indicator off after 500 ms
ros2 launch toio_gazebo simulation.launch.py led_duration_ms:=500
```

![The indicator of the cube blinking through a sequence of colors](image/toio_led.gif)

### Notes on the LED and the sound effects

The lamp is the ball on the underside of the cube, as
[the specification](https://toio.github.io/toio-spec/en/docs/ble_light/)
describes it, and is driven by the `ToioLedSystem` Gazebo plugin of this
package. Only the cap below the flat underside is visible, so it is best seen
from a low angle.

The plugin drives a light along with the visual, so the mat picks up the color
of the lamp rather than the lamp only lighting up itself. As on the real cube,
the mat right under it takes a strong spot about the size of the lamp, which
fades out over roughly the size of the cube. Set `led_light_intensity:=0` to
leave the mat alone.
The way the plugin drives the material and the light through
`components::VisualCmd` and `components::LightCmd` follows
[gz_sim_led_plugin](https://github.com/jasmeet0915/gz_sim_led_plugin)
(Apache License 2.0).

Gazebo has no audio output, so `toio_sound_node.py` plays the sound effects on
the host through `aplay`. If `aplay` is missing, the requests are only logged.
The sound effects of the real cube cannot be redistributed here, so each id is
approximated by a synthesized motif rather than the original sound.

## Multi-robot simulation

`toio_multi_simulation.launch.py` spawns two toio robots (`toio1` and `toio2`).

```bash
ros2 launch toio_gazebo toio_multi_simulation.launch.py
```

Each robot is separated by ROS namespace and TF frame prefix.

| robot | cmd_vel | joint_states | led | sound | TF |
| --- | --- | --- | --- | --- | --- |
| toio1 | `/toio1/cmd_vel` | `/toio1/joint_states` | `/toio1/toio/led` | `/toio1/toio/sound` | `map -> toio1/center` |
| toio2 | `/toio2/cmd_vel` | `/toio2/joint_states` | `/toio2/toio/led` | `/toio2/toio/sound` | `map -> toio2/center` |

An example to move each robot independently:

```bash
ros2 topic pub -r 10 /toio1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
ros2 topic pub -r 10 /toio2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
```

An example to light each robot in a different color:

```bash
ros2 topic pub --once /toio1/toio/led std_msgs/msg/ColorRGBA '{r: 1.0, g: 0.0, b: 0.0, a: 1.0}'
ros2 topic pub --once /toio2/toio/led std_msgs/msg/ColorRGBA '{r: 0.0, g: 0.0, b: 1.0, a: 1.0}'
```

To spawn an additional robot into a running simulation, use `spawn_toio.launch.py`:

```bash
ros2 launch toio_gazebo spawn_toio.launch.py namespace:=toio3 robot_name:=toio3 frame_prefix:=toio3/ x_pose:=0.145 y_pose:=-0.095
```
