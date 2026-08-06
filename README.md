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

## Multi-robot simulation

`toio_multi_simulation.launch.py` spawns two toio robots (`toio1` and `toio2`).

```bash
ros2 launch toio_gazebo toio_multi_simulation.launch.py
```

Each robot is separated by ROS namespace and TF frame prefix.

| robot | cmd_vel | joint_states | TF |
| --- | --- | --- | --- |
| toio1 | `/toio1/cmd_vel` | `/toio1/joint_states` | `map -> toio1/center` |
| toio2 | `/toio2/cmd_vel` | `/toio2/joint_states` | `map -> toio2/center` |

An example to move each robot independently:

```bash
ros2 topic pub -r 10 /toio1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
ros2 topic pub -r 10 /toio2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}'
```

To spawn an additional robot into a running simulation, use `spawn_toio.launch.py`:

```bash
ros2 launch toio_gazebo spawn_toio.launch.py namespace:=toio3 robot_name:=toio3 frame_prefix:=toio3/ x_pose:=0.145 y_pose:=-0.095
```
